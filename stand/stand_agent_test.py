import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
import matplotlib.pyplot as plt
import pybullet as p
import sys, os
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
gym.register(
    id="stand-v0",
    entry_point="stand.stand_env:stand", 
)


def plot_gradient_line(x, y, z, ax, cmap_name):
    """將線條切成小片段，根據索引賦予漸層顏色（前面灰、後面鮮豔）"""
    points = np.array([x, y, z]).T.reshape(-1, 1, 3)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    
    # 建立 Colormap (例如胸部用藍色系、頭部用橘紅系)
    cmap = plt.get_cmap(cmap_name)
    
    # 建立 LineCollection，alpha 與顏色隨著時間遞進
    # 前面 (i=0) 比較灰暗/透明，後面 (i=end) 比較鮮豔飽和
    colors = [cmap(i / len(segments)) for i in range(len(segments))]
    
    lc = Line3DCollection(segments, colors=colors, linewidths=2.5)
    ax.add_collection3d(lc)




# 加载训练好的模型
file_name = "/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/stand/models/humanoid_stand_ppo.zip"
pkl_name = "/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/stand/pkls/humanoid_vecnormalize.pkl"

model = PPO.load(file_name)

env = gym.make(
    "stand-v0",
    render_mode="human"
)
# 加载归一化参数
env = DummyVecEnv([lambda: env])  # 包装为向量环境
env = VecNormalize.load(pkl_name, env)
env.training = False
env.norm_reward = False

# 运行测试
episode_rewards = []
chest_pos = []
head_pos = []
dDT = 1.0 / 240.0
for ep in range(10):  # 测试10个episode
    obs = env.reset()
    total_reward = 0
    
    while True:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        
        # 记录关键数据
        total_reward += reward[0]

        # 获取物理状态

        head_pos.append(info[0]["human_head"])

        if done[0]:
            # 计算行走速度
            
            print(f"Episode {ep+1}:")
            print(f"  - Total Reward: {total_reward:.1f}")
            print(f"  - Survival Time: {len(chest_pos)*4*dDT:.1f}s")
            episode_rewards.append(total_reward)
            

            fig = plt.figure(figsize=(8, 6))
            ax = fig.add_subplot(111, projection='3d')

            min_x, max_x = 0,0
            min_y, max_y = 0,0
            al =0
            point = head_pos 
            for i,sp in enumerate(point):
                al = ((i+1)/len(point)) ** 3
                ax.scatter(sp[0], sp[1], sp[2], color='red',alpha=al, s=100)
                min_x = min(sp[0],min_x)
                max_x = max(sp[0],max_x)
                min_y = min(sp[0],min_y)
                max_y = max(sp[0],max_y)
 

            x = np.linspace(min_x, max_x, 100)
            y = np.linspace(min_y, max_y, 100)
            xx, yy = np.meshgrid(x, y)

            zz = np.full_like(xx, 1)
            ax.plot_surface(xx, yy, zz, color='cyan', alpha=0.5, label='Z = 3 Plane')

            ax.set_xlabel('X Axis')
            ax.set_ylabel('Y Axis')
            ax.set_zlabel('Z Axis')
            ax.set_xlim(min_x, max_x)
            ax.set_ylim(min_y, max_y)
            ax.set_zlim(0, 1.5)

            ax.legend()
            plt.show()
            time.sleep(3)
            break


env.close()


