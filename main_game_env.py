import math
from typing import Any

import numpy as np

import gymnasium as gym
from gymnasium import logger, spaces
from gymnasium.envs.classic_control import utils
from gymnasium.error import DependencyNotInstalled
from gymnasium.vector import AutoresetMode, VectorEnv
from gymnasium.vector.utils import batch_space
import pybullet as p
import random
import pybullet_data




class door_out(gym.Env):
    '''
    最終地圖
    '''
    metadata = {
        "render_modes": ["human", "no"],
    }

    def __init__(self, sutton_barto_reward: bool = False, render_mode: str | None = None):
        if render_mode == "human":
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
            
        p.setAdditionalSearchPath(pybullet_data.getDataPath())


        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(30,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(72,), dtype=np.float32
        )

        self.human = None
        self.cardId = None
        self.readerId = None
        self.doorId = None
        self.pick_constraint = None
        self.room_size = 5

        self.door_timer = 0
        self.pick_up_or_not = 0
        self.bi = 0

    #獎勵和遊戲內邏輯
    def step(self, action):
        reward = 0
        # 1. 執行動作：將 30 維的 action 套用到機器人的 30 個關節上
        for i in range(30):
            target_vel = float(action[i]) * 5.0  # 放大係數可依需求調整
            p.setJointMotorControl2(
                bodyUniqueId=self.human,
                jointIndex=i,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=target_vel,
                force=50
            )
            
        # 2. 更新環境邏輯 (讀卡機與門)
        closest_points = p.getClosestPoints(self.cardId, self.readerId, 1.0)
        if closest_points and closest_points[0][8] < 0.5:
            self.door_timer = 20 * 240
            if self.bi == 0:
                reward +=50
                self.bi = 1
        if self.door_timer > 0:
            self.door_timer -= 1
            p.setJointMotorControl2(bodyUniqueId=self.doorId, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=5.0, force=500)
        else:
            p.setJointMotorControl2(bodyUniqueId=self.doorId, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=0.0, force=500)

        # 3. 檢查碰撞並自動抓取卡片 (維持你原本的邏輯)
        if self.pick_constraint is None:
            contact_points = p.getContactPoints(bodyA=self.human, bodyB=self.cardId, linkIndexA=29)
            if contact_points:
                rh_state = p.getLinkState(self.human, 29)
                card_state = p.getBasePositionAndOrientation(self.cardId)
                
                inv_pos, inv_orn = p.invertTransform(rh_state[0], rh_state[1])
                rel_pos, rel_orn = p.multiplyTransforms(inv_pos, inv_orn, card_state[0], card_state[1])
                
                self.pick_constraint = p.createConstraint(
                    parentBodyUniqueId=self.human, parentLinkIndex=29,
                    childBodyUniqueId=self.cardId, childLinkIndex=-1,
                    jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
                    parentFramePosition=rel_pos, childFramePosition=[0, 0, 0],
                    parentFrameOrientation=rel_orn, childFrameOrientation=[0, 0, 0]
                )
                p.setCollisionFilterPair(self.human, self.cardId, -1, -1, 0)
                self.pick_up_or_not = 1
                reward += 50

        # 4. 單步模擬
        p.stepSimulation()
        
        # 5. 計算獎勵 (Reward) 與 終止條件 (Done / Terminated)
        observation = self._get_observation()
        
        human_pos = p.getBasePositionAndOrientation(self.human)[0]
        card_pos = p.getBasePositionAndOrientation(self.cardId)[0]
        
        # 範例獎勵設計：
        # - 鼓勵機器人靠近卡片、卡片靠近讀卡機、機器人走過門口 (X > 5)
        reward -= 0.01  # 時間懲罰，鼓勵快點完成
        
        terminated = False
        truncated = False
        
        if human_pos[0] > 5.0:  # 通關條件
            reward += 100.0
            terminated = True
            
        info = {}
        return observation, reward, terminated, truncated, info
    #重製參數和世界物品
    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict | None = None,
    ):
        super().reset(seed=seed)
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        p.loadURDF("plane.urdf")
        self.set_up()
        self.pick_constraint = None
        self.door_timer = 0
        self.bi = 0
        self.pick_up_or_not = 0

        observation = self._get_observation()
        info = {}

        return observation, info

    #創建世界物品
    def set_up(self):

        self.human = p.loadURDF("/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/humanoid3_colli.urdf",[0,0,1])
        joint_NameToId = {}
        num_joints = p.getNumJoints(self.human)
        for i in range(num_joints):
            info = p.getJointInfo(self.human, i)
            joint_name = info[1].decode('utf-8')
            joint_NameToId[joint_name] = i


        wall_col1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, self.room_size, 2])
        wall_col2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[self.room_size, 0.2, 2])
        wall_col3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, (self.room_size/2)-0.25, 2])

        wallId_1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col1, basePosition=[-self.room_size, 0, 0])
        wallId_2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col2, basePosition=[0, -self.room_size, 0])
        offf = (self.room_size/2)+0.25
        wallId_3_L = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col3, basePosition=[self.room_size,offf, 0])
        wallId_3_R = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col3, basePosition=[self.room_size,-offf, 0])
        wallId_4 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col2, basePosition=[0, self.room_size, 0])

        direction_wall = random.choice([self.room_size-0.2,-self.room_size+0.2])
        wall_size_radom = random.uniform(-self.room_size+0.3,self.room_size-0.3)
        avoid_door = wall_size_radom if abs(wall_size_radom) > 1 else 1.5

        respon_area_reader_v = [wall_size_radom,direction_wall, 1]
        respon_area_reader_h = [direction_wall,avoid_door, 1]
        reader_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])
        reader_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2], rgbaColor=[0.2, 0.2, 0.8, 1]) # 藍色
        self.readerId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=reader_col, baseVisualShapeIndex=reader_vis, basePosition=random.choice([respon_area_reader_v,respon_area_reader_h]))

        respon_area_card = [random.uniform(-self.room_size+1,self.room_size-1), random.uniform(-self.room_size+1,self.room_size-1), 0.5]
        card_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.02])
        card_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.02], rgbaColor=[0.8, 0.2, 0.2, 1]) # 紅色
        self.cardId = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=card_col, baseVisualShapeIndex=card_vis, basePosition=respon_area_card)


        door_link_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.5, 1.0])
        door_link_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.5, 1.0], rgbaColor=[0.5, 0.5, 0.5, 1]) # 灰色


        self.doorId = p.createMultiBody(
            baseMass=0,                        # 基座固定
            baseCollisionShapeIndex=-1,        # 基座沒有形狀
            baseVisualShapeIndex=-1,           # 基座沒有外觀
            basePosition=[self.room_size-0.22, 0, 1],        # 整體的起始位置
            
            linkMasses=[10.0],
            linkCollisionShapeIndices=[door_link_col],
            linkVisualShapeIndices=[door_link_vis],
            linkPositions=[[0, 0, 0]],                  # 相對基座的位置
            linkOrientations=[[0, 0, 0, 1]],            # 相對基座的姿態
            linkInertialFramePositions=[[0, 0, 0]],     # 慣性中心位置
            linkInertialFrameOrientations=[[0, 0, 0, 1]],# 慣性中心姿態
            linkParentIndices=[0],                      # 父節點索引 (0 代表連接到基座)
            linkJointTypes=[p.JOINT_PRISMATIC],         # 關鍵：滑軌關節
            linkJointAxis=[[0, 0, 1]]                     # 沿著 Y 軸左右移動
        )

        return 0
    #得到觀察值
    def _get_observation(self):
        # 收集 30 個關節的角度與速度
        joint_angles = []
        joint_velocities = []
        for i in range(30):
            state = p.getJointState(self.human, i)
            joint_angles.append(state[0])
            joint_velocities.append(state[1])
            
        # 收集各物件 XYZ 座標
        human_pos, _ = p.getBasePositionAndOrientation(self.human)
        card_pos, _ = p.getBasePositionAndOrientation(self.cardId)
        reader_pos, _ = p.getBasePositionAndOrientation(self.readerId)
        door_pos, _ = p.getBasePositionAndOrientation(self.doorId)
        
        obs = np.concatenate([
            joint_angles, 
            joint_velocities, 
            human_pos, 
            card_pos, 
            reader_pos, 
            door_pos
        ]).astype(np.float32)
        
        return obs

    #關閉pybullet連線
    def close(self):
        p.disconnect()

class walk(gym.Env):
    '''
    走路地圖
    '''
    metadata = {
        "render_modes": ["human", "no"],
    }

    def __init__(self, sutton_barto_reward: bool = False, render_mode: str | None = None):
        if render_mode == "human":
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
            
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(30,), dtype=np.float32
        )
        #observation: 30 joint,target relative x and y,
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(63,), dtype=np.float32
        )

        self.human = None
        self.target = [0,0]
        self.min_distance = 0
    #獎勵和遊戲內邏輯
    def step(self, action):
        reward = 0
        # 1. 執行動作：將 30 維的 action 套用到機器人的 30 個關節上
        for i in range(30):
            target_vel = float(action[i]) * 5.0  # 放大係數可依需求調整
            p.setJointMotorControl2(
                bodyUniqueId=self.human,
                jointIndex=i,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=target_vel,
                force=50
            )
            


        # 4. 單步模擬
        p.stepSimulation()
        
        # 5. 計算獎勵 (Reward) 與 終止條件 (Done / Terminated)
        observation = self._get_observation()
        
        human_pos = p.getBasePositionAndOrientation(self.human)[0]
        human_pos = np.array(human_pos)
        relative_pos = self.target - human_pos
        relative_distance = np.sum(relative_pos[:2]**2)
        # 範例獎勵設計：
        # - 鼓勵機器人靠近卡片、卡片靠近讀卡機、機器人走過門口 (X > 5)
        reward -= 0.01  # 時間懲罰，鼓勵快點完成
        
        terminated = False
        truncated = False
        
        if relative_distance < self.min_distance:  # 通關條件
            reward += 10
            self.min_distance = relative_distance
        if relative_distance < 1:
            terminated = True
            
        info = {}
        return observation, reward, terminated, truncated, info
    #重製參數和世界物品
    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict | None = None,
    ):
        super().reset(seed=seed)
        p.resetSimulation()
        self.set_up()
        p.setGravity(0, 0, -9.8)
        p.loadURDF("plane.urdf")

        observation = self._get_observation()
        info = {}

        return observation, info

    #創建世界物品
    def set_up(self):

        xx =random.uniform(2,5)*random.choice([-1,1])
        yy =random.uniform(2,5)*random.choice([-1,1])
        self.target = np.array([xx,yy,0])
        self.min_distance = np.sum(self.target[:2]**2)
        self.human = p.loadURDF("/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/humanoid3_colli.urdf",[0,0,1])
        joint_NameToId = {}
        num_joints = p.getNumJoints(self.human)
        for i in range(num_joints):
            info = p.getJointInfo(self.human, i)
            joint_name = info[1].decode('utf-8')
            joint_NameToId[joint_name] = i
        return 0
    #得到觀察值
    def _get_observation(self):
        # 收集 30 個關節的角度與速度
        joint_angles = []
        joint_velocities = []
        for i in range(30):
            state = p.getJointState(self.human, i)
            joint_angles.append(state[0])
            joint_velocities.append(state[1])
            
        # 收集各物件 XYZ 座標
        h_pos, _ = p.getBasePositionAndOrientation(self.human)
        h_pos = np.array(h_pos)
        relative_pos = self.target - h_pos
        
        obs = np.concatenate([
            joint_angles, 
            joint_velocities, 
            relative_pos
        ]).astype(np.float32)
        
        return obs

    #關閉pybullet連線
    def close(self):
        p.disconnect()

class stand(gym.Env):
    '''
    站立地圖 TODO
    '''
    metadata = {
        "render_modes": ["human", "no"],
    }

    def __init__(self, sutton_barto_reward: bool = False, render_mode: str | None = None):
        if render_mode == "human":
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
            
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(30,), dtype=np.float32
        )
        # observation: 30 joint angles, 30 joint velocities, chest(3), right_leg(3), left_leg(3) = 69 dims
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(69,), dtype=np.float32
        )

        self.human = None
        self.human_chest = None
        self.human_left_leg = None
        self.human_right_leg = None

        self.chest_link_index = 15      #TODO change it to WAIST_LINK2 index
        self.left_leg_link_index = 11   #TODO change it to r_ankle index
        self.right_leg_link_index = 5   #TODO change it to l_ankle index

        self.limit = 10000
        self.max_time = 100 * 240

    # 獎勵和遊戲內邏輯
    def step(self, action):
        terminated = False
        truncated = False
        reward = 0
        self.max_time -= 1
        if self.max_time <= 0:
            truncated = True

        # 1. 執行動作：將 30 維的 action 套用到機器人的 30 個關節上
        for i in range(30):
            target_vel = float(action[i]) * 5.0  # 放大係數可依需求調整
            p.setJointMotorControl2(
                bodyUniqueId=self.human,
                jointIndex=i,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=target_vel,
                force=50,
                physicsClientId=self.client
            )

        # 4. 單步模擬
        p.stepSimulation(physicsClientId=self.client)
        
        # 5. 計算獎勵 (Reward) 與 終止條件 (Done / Terminated)
        observation = self._get_observation()
        
        human_pos = p.getBasePositionAndOrientation(self.human, physicsClientId=self.client)[0]
        human_pos = np.array(human_pos)
        reward += 0.01

        link_state = p.getLinkState(self.human, self.chest_link_index, physicsClientId=self.client)
        self.human_chest = np.array(link_state[0])
        link_state = p.getLinkState(self.human, self.left_leg_link_index, physicsClientId=self.client)
        self.human_left_leg = np.array(link_state[0])
        link_state = p.getLinkState(self.human, self.right_leg_link_index, physicsClientId=self.client)
        self.human_right_leg = np.array(link_state[0])

        if self.human_chest[2] < 0.8:
            reward-=1
            self.limit -= 1
        if self.limit <= 0:
            terminated = True
        if self.human_right_leg[2] < 0.25 and self.human_left_leg[2] < 0.25:
            reward += 1


        # contact_points = p.getContactPoints(bodyA=self.human, physicsClientId=self.client)
        # allowed_contact_links = {self.left_leg_link_index, self.right_leg_link_index}
        
        # for contact in contact_points:
        #     link_index = contact[3]
        #     if link_index not in allowed_contact_links:
        #         reward -= 100
        #         terminated = True
        #         break
        info = {"limit":self.limit}
        return observation, reward, terminated, truncated, info

    # 重製參數和世界物品
    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict | None = None,
    ):
        super().reset(seed=seed)
        # 【修正重點】加上 physicsClientId，確保是對當前環境的連線進行重置
        p.resetSimulation(physicsClientId=self.client)
        self.set_up()
        p.setGravity(0, 0, -9.8, physicsClientId=self.client)
        p.loadURDF("plane.urdf", physicsClientId=self.client)

        observation = self._get_observation()
        info = {}

        return observation, info

    # 創建世界物品
    def set_up(self):
        self.limit = 10000
        self.max_time = 100 * 240  # 確保每次 reset 時時間倒數也能重設
        self.human = p.loadURDF(
            "/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/humanoid3_colli.urdf",
            [0, 0, 1],
            physicsClientId=self.client
        )
        joint_NameToId = {}
        num_joints = p.getNumJoints(self.human, physicsClientId=self.client)
        for i in range(num_joints):
            info = p.getJointInfo(self.human, i, physicsClientId=self.client)
            joint_name = info[1].decode('utf-8')
            joint_NameToId[joint_name] = i
        
        for i in range(p.getNumJoints(self.human, physicsClientId=self.client)):
            info = p.getJointInfo(self.human, i, physicsClientId=self.client)
            child_name = info[12].decode('utf-8')
            if child_name == "WAIST_LINK2":
                self.chest_link_index = i
            elif child_name == "l_ankle":
                self.left_leg_link_index = i
            elif child_name == "r_ankle":
                self.right_leg_link_index = i
                
        
        link_state = p.getLinkState(self.human, self.chest_link_index, physicsClientId=self.client)
        self.human_chest = np.array(link_state[0])
        link_state = p.getLinkState(self.human, self.left_leg_link_index, physicsClientId=self.client)
        self.human_left_leg = np.array(link_state[0])
        link_state = p.getLinkState(self.human, self.right_leg_link_index, physicsClientId=self.client)
        self.human_right_leg = np.array(link_state[0])
        return 0

    # 得到觀察值
    def _get_observation(self):
        joint_angles = []
        joint_velocities = []
        for i in range(30):
            state = p.getJointState(self.human, i, physicsClientId=self.client)
            joint_angles.append(state[0])
            joint_velocities.append(state[1])
            
        h_pos, _ = p.getBasePositionAndOrientation(self.human, physicsClientId=self.client)
        h_pos = np.array(h_pos)

        obs = np.concatenate([
            joint_angles, 
            joint_velocities, 
            self.human_chest,
            self.human_right_leg,
            self.human_left_leg
        ]).astype(np.float32)
        
        return obs

    # 關閉 pybullet 連線
    def close(self):
        if self.client is not None:
            p.disconnect(physicsClientId=self.client)
            self.client = None