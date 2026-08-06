#TODO 現在要來用機械手臂訓練抓取卡片
import gymnasium as gym
import pybullet as p
import pybullet_data
from gymnasium import logger, spaces
import numpy as np
import random




class arm_grap(gym.Env):
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
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(28,), dtype=np.float32
        )

        self.cardId = None
        self.car = None
        self.arm = None
        self.room_size = 5
        self.max_time = 240 * 3
        self.pre_distance = 999
        self.config ={
                "gravity":-9.8,
                "startPos":[0,0,0.2],
            }

    #獎勵和遊戲內邏輯
    def step(self, action):
        reward = 0
        self.max_time -=1
        #動作邊號，關節編號，最大速度
        action_joinID_maxV = [
            [0,   0,   2.175],
            [1,   1,   2.175],
            [2,   2,   2.175],
            [3,   3,   2.175],
            [4,   4,   2.61],
            [5,   5,   2.61],
            [6,   6,   2.61],
            [7,   9,   0.2],
        ]
        for a,j,v in action_joinID_maxV:
            target_vel = float(action[a]) * v * 5 # 放大係數可依訓練速需求調整
            p.setJointMotorControl2(
                bodyUniqueId=self.arm,
                jointIndex=j,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=target_vel,
                force=50
            )

        # {'panda_joint1': 0,
        # 'panda_joint2': 1,
        # 'panda_joint3': 2,
        # 'panda_joint4': 3,
        # 'panda_joint5': 4,
        # 'panda_joint6': 5,
        # 'panda_joint7': 6,
        # 'panda_joint8': 7,
        # 'panda_hand_joint': 8,
        # 'panda_finger_joint1': 9,
        # 'panda_finger_joint2': 10,
        # 'panda_grasptarget_hand': 11}
        # 4. 單步模擬
        for _ in range(5):
            p.stepSimulation()
        terminated ,truncated = False, False
        # 5. 計算獎勵 (Reward) 與 終止條件 (Done / Terminated)
        observation = self._get_observation()
        # tip_pos = p.getLinkState(self.arm,9)[0]
        grap_pos = p.getLinkState(self.arm,11)[0]
        card_pos,_ = p.getBasePositionAndOrientation(self.cardId)
        pos1 = np.array(grap_pos)
        pos2 = np.array(card_pos)
        distance = np.linalg.norm(pos1 - pos2)
        sigma_near = 0.2
        sigma_mid  = 0.5
        sigma_far  = 1
        #手臂伸到物品reward
        reward -= (1.0 - np.exp(-(distance ** 2) / (2 * sigma_near ** 2))) *0.5
        reward -= (1.0 - np.exp(-(distance ** 2) / (2 * sigma_mid ** 2)))  *0.3
        reward -= (1.0 - np.exp(-(distance ** 2) / (2 * sigma_far ** 2)))  *0.2
        #當手指接近目標後，啟動速度懲罰
        if distance<0.1:
            joint_velocity = []
            for i in [0,1,2,3,4,5,6,9]:
                joint_state = p.getJointState(self.arm, i)
                joint_velocity.append(joint_state[1])
            joint_vel = np.array(joint_velocity)
            reward -= 0.001 * np.sum(joint_vel**2)
        #控制懲罰
        reward -= 0.0005 * np.sum(action**2)

        #距離變化量懲罰
        reward -= (distance - self.pre_distance) * 5
        self.pre_distance = distance
        
        #兩手指同時接觸卡片
        contact_points_1 = p.getContactPoints(bodyA=self.arm, bodyB=self.cardId, linkIndexA=9)
        contact_points_2 = p.getContactPoints(bodyA=self.arm, bodyB=self.cardId, linkIndexA=10)
        if len(contact_points_1) > 0:
            reward += 0.5
        if len(contact_points_2) > 0:
            reward += 0.5
        #卡片抬起判定

        if card_pos[2]>0.05:
            reward +=1
        # if distance<0.1:
        #     reward+=50
        #     terminated = True
        if self.max_time == 0:
            truncated = True
        #在pybullet上顯示reward
        if hasattr(self, 'text_id') and self.text_id is not None:
            p.removeUserDebugItem(self.text_id)
        text_str = f"Reward: {reward:.2f} | Time Left: {self.max_time}"
        self.text_id = p.addUserDebugText(
            text=text_str,
            textPosition=[0, 0, 1], 
            textColorRGB=[0, 0, 0],
            textSize=1.5,
            lifeTime=0  # lifeTime=0 代表它會持續存在，直到被我們手動 remove 換掉
        )
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
        p.setGravity(0, 0, self.config["gravity"])
        p.loadURDF("plane.urdf")
        self.set_up()
        grap_pos = p.getLinkState(self.arm,11)[0]
        card_pos,_ = p.getBasePositionAndOrientation(self.cardId)
        self.pre_distance = np.linalg.norm(
            np.array(grap_pos)-np.array(card_pos)
        )

        self.max_time = 240*3
        observation = self._get_observation()
        info = {}

        return observation, info

    #創建世界物品
    def set_up(self):


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
        card_x =random.uniform(0.5,0.8)*random.choice([-1,1]) 
        card_y =random.uniform(0.5,0.8)*random.choice([-1,1]) 
        respon_area_card = [card_x, card_y, 0.01]
        card_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
        card_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=[0.8, 0.2, 0.2, 1]) # 紅色
        self.cardId = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=card_col, baseVisualShapeIndex=card_vis, basePosition=respon_area_card)

#````````````````````````````````````````````````````````````
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        #車子
        self.car = p.loadURDF("/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/car/husky/husky.urdf",
                            self.config["startPos"], 
                            startOrientation)
        #機械手臂
        self.arm = p.loadURDF("/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/car/franka_panda/panda.urdf",
                            [0,0,0.5], startOrientation)
        constraint_id = p.createConstraint(
            parentBodyUniqueId=self.car,
            parentLinkIndex=-1,  # -1 通常代表車體基底 (base_link)
            childBodyUniqueId=self.arm,
            childLinkIndex=-1,  # 如果是單一檔案物件通常也是 -1
            jointType=p.JOINT_FIXED,  # 設為固定關節 (像焊接一樣)
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0.5],  # 在父物件（車子）身上的固定點位置
            childFramePosition=[0, 0, 0],  # 在子物件身上的對齊中心點
            parentFrameOrientation=[0, 0, 0, 1],
            childFrameOrientation=[0, 0, 0, 1],
        )


        return 0
    #得到觀察值
    def _get_observation(self):
        # 收集 30 個關節的角度與速度

        # 收集各物件 XYZ 座標
        card_pos, _ = p.getBasePositionAndOrientation(self.cardId)
        car_pos, _ = p.getBasePositionAndOrientation(self.car)
        arm_pos, _ = p.getBasePositionAndOrientation(self.arm)
        joint_position = []
        joint_velocity = []
        for i in [0,1,2,3,4,5,6,9]:
            joint_state = p.getJointState(self.arm, i)
            joint_position.append(joint_state[0]) # 該關節當前的角度 (position)
            joint_velocity.append(joint_state[1]) # 該關節當前的速度 (velocity)
        grap_pos = p.getLinkState(self.arm,11)[0]
        obs = np.concatenate([
            joint_position,
            joint_velocity,
            grap_pos,
            card_pos,
            car_pos,
            arm_pos,
        ]).astype(np.float32)
        
        return obs

    #關閉pybullet連線
    def close(self):
        if p.isConnected():
            p.disconnect()