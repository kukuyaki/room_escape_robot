'''
手臂抓取卡片的模型測試
'''

import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
import matplotlib.pyplot as plt
import sys
import os
import time



sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
gym.register(
    id="arm_bi-v0",
    entry_point="car.env_arm_bi:arm_bi", 
)

# 加载训练好的模型
model = PPO.load("./car/models/car_bi_1")
env = gym.make(
    "arm_bi-v0",
    render_mode="human",  # 开启实时渲染
)

# 加载归一化参数
env = DummyVecEnv([lambda: env])  # 包装为向量环境
env = VecNormalize.load("./car/pkls/car_bi_vecnormalize.pkl", env)
env.training = False
env.norm_reward = False

# 运行测试
episode_rewards = []
joint_angle_history = []
velocity_history = []

for ep in range(10):  # 测试10个episode
    obs = env.reset()
    total_reward = 0
    positions = []
    
    while True:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        time.sleep(0.01)
        if done[0]:
            # 计算行走速度
            print(f"Episode {ep+1}:")
            print(f"  - Total Reward: {total_reward:.1f}")
            break


env.close()



