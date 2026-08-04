#TODO 現在要來用機械手臂訓練抓取卡片

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
            low=-np.inf, high=np.inf, shape=(51,), dtype=np.float32
        )

        self.cardId = None

        self.room_size = 5



    #獎勵和遊戲內邏輯
    def step(self, action):
        reward = 0
            


        # 4. 單步模擬
        p.stepSimulation()
        
        # 5. 計算獎勵 (Reward) 與 終止條件 (Done / Terminated)
        observation = self._get_observation()
        
        card_pos = p.getBasePositionAndOrientation(self.cardId)[0]

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

        respon_area_card = [random.uniform(-self.room_size+1,self.room_size-1), random.uniform(-self.room_size+1,self.room_size-1), 0.5]
        card_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.02])
        card_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.02], rgbaColor=[0.8, 0.2, 0.2, 1]) # 紅色
        self.cardId = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=card_col, baseVisualShapeIndex=card_vis, basePosition=respon_area_card)



        return 0
    #得到觀察值
    def _get_observation(self):
        # 收集 30 個關節的角度與速度

        # 收集各物件 XYZ 座標
        card_pos, _ = p.getBasePositionAndOrientation(self.cardId)

        obs = np.concatenate([
            card_pos
        ]).astype(np.float32)
        
        return obs

    #關閉pybullet連線
    def close(self):
        p.disconnect()