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
            low=-1.0, high=1.0, shape=(7,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(9,), dtype=np.float32
        )

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


        # 4. 單步模擬
        p.stepSimulation()
        
        # 5. 計算獎勵 (Reward) 與 終止條件 (Done / Terminated)
        observation = self._get_observation()
        
        card_pos = p.getBasePositionAndOrientation(self.cardId)[0]
        
        # 範例獎勵設計：
        # - 鼓勵機器人靠近卡片、卡片靠近讀卡機、機器人走過門口 (X > 5)
        reward -= 0.01  # 時間懲罰，鼓勵快點完成
        
        terminated = False
        truncated = False
    
        reward += 100.0
        terminated = False
            
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
        self.door_timer = 0
        self.bi = 0
        self.pick_up_or_not = 0

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

        # 收集各物件 XYZ 座標
        card_pos, _ = p.getBasePositionAndOrientation(self.cardId)
        reader_pos, _ = p.getBasePositionAndOrientation(self.readerId)
        door_pos, _ = p.getBasePositionAndOrientation(self.doorId)
        
        obs = np.concatenate([ 
            card_pos, 
            reader_pos, 
            door_pos
        ]).astype(np.float32)
        
        return obs

    #關閉pybullet連線
    def close(self):
        p.disconnect()