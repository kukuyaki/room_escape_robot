
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
            low=-np.inf, high=np.inf, shape=(72,), dtype=np.float32
        )

        self.human = None
        self.human_chest = None
        self.human_left_leg = None
        self.human_right_leg = None
        self.human_head = None

        self.chest_link_index = 0     #TODO change it to WAIST_LINK2 index
        self.left_leg_link_index = 0  #TODO change it to r_ankle index
        self.right_leg_link_index = 0   #TODO change it to l_ankle index
        self.head_link_index = 0

        self.limit = 10000
        self.max_time = 100 * 240
        self.episode_count = 0
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
        link_state = p.getLinkState(self.human, self.head_link_index, physicsClientId=self.client)
        self.human_head = np.array(link_state[0])
        link_state = p.getLinkState(self.human, self.left_leg_link_index, physicsClientId=self.client)
        self.human_left_leg = np.array(link_state[0])
        link_state = p.getLinkState(self.human, self.right_leg_link_index, physicsClientId=self.client)
        self.human_right_leg = np.array(link_state[0])
        reward +=1
        if self.human_chest[2] < 0.8:
            reward-=0.1
            self.limit -= 0.1
        else:
            reward+=10
        if self.human_head[2] < 1.2:
            reward-=0.1
            self.limit -= 0.1
        else:
            reward+=10
        if self.limit <= 0:
            terminated = True
        # if self.human_right_leg[2] < 0.25 and self.human_left_leg[2] < 0.25:
        #     reward += 1

        contact_points = p.getContactPoints(bodyA=self.human, physicsClientId=self.client)
        allowed_contact_links = {self.left_leg_link_index, self.right_leg_link_index}
        for contact in contact_points:
            link_index = contact[3]
            if link_index not in allowed_contact_links:
                reward -= 10
                terminated = True
                break
        
        info = {"limit":self.limit,"max_time":self.max_time}
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
        info = {"episode_count":self.episode_count}

        return observation, info

    # 創建世界物品
    def set_up(self):
        self.episode_count += 1
        self.limit = 10000
        self.max_time = 100 * 240  # 確保每次 reset 時時間倒數也能重設
        self.human = p.loadURDF(
            "/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/humanoid3_colli.urdf",
            [0, 0, 0.8],
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
            elif child_name == "torso":
                self.head_link_index = i
                
        link_state = p.getLinkState(self.human, self.head_link_index, physicsClientId=self.client)
        self.human_head = np.array(link_state[0])
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
            self.human_head,
            self.human_left_leg
        ]).astype(np.float32)
        
        return obs

    # 關閉 pybullet 連線
    def close(self):
        if self.client is not None:
            p.disconnect(physicsClientId=self.client)
            self.client = None