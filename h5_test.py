import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
import matplotlib.pyplot as plt

def visualize_gait(joint_angles: np.ndarray):
    """可视化关节角度时序变化"""
    plt.figure(figsize=(12, 6))
    labels = [
        'abdomen_z', 'abdomen_y', 'abdomen_x',
        'right_hip_x', 'right_hip_z', 'right_hip_y', 'right_knee',
        'left_hip_x', 'left_hip_z', 'left_hip_y', 'left_knee',
        'right_shoulder1', 'right_shoulder2', 'right_elbow',
        'left_shoulder1', 'left_shoulder2', 'left_elbow'
    ]
    
    for i in range(17):
        plt.plot(joint_angles[:, i], label=labels[i] if i < len(labels) else f'Joint {i}')
    
    plt.title("Joint Angles During Walking")
    plt.xlabel("Time Step")
    plt.ylabel("Angle (rad)")
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.savefig("./gait_analysis.png")
    plt.close()

# 加载训练好的模型
model = PPO.load("humanoid_walk_ppo")
env = gym.make(
    "Humanoid-v5",
    render_mode="human",  # 开启实时渲染
    exclude_current_positions_from_observation=True
)

# 加载归一化参数
env = DummyVecEnv([lambda: env])  # 包装为向量环境
env = VecNormalize.load("humanoid_vecnormalize.pkl", env)
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
        
        # 记录关键数据
        total_reward += reward[0]
        joint_angle_history.append(action[0])  # 记录动作（关节角度）
        
        # 获取物理状态
        qpos = env.envs[0].unwrapped.data.qpos
        positions.append(qpos[0:2].copy())  # 记录x,y位置
        
        if done[0]:
            # 计算行走速度
            positions = np.array(positions)
            dx = positions[-1, 0] - positions[0, 0]
            dt = len(positions) * env.envs[0].unwrapped.dt
            velocity_history.append(dx / dt)
            
            print(f"Episode {ep+1}:")
            print(f"  - Total Reward: {total_reward:.1f}")
            print(f"  - Average Speed: {velocity_history[-1]:.2f} m/s")
            print(f"  - Distance: {dx:.2f} meters")
            print(f"  - Survival Time: {len(positions)*env.envs[0].unwrapped.dt:.1f}s")
            episode_rewards.append(total_reward)
            break

# 可视化分析
visualize_gait(np.array(joint_angle_history))

# 生成速度分布图
plt.figure(figsize=(8, 4))
plt.hist(velocity_history, bins=10, edgecolor='black')
plt.title("Walking Speed Distribution")
plt.xlabel("Speed (m/s)")
plt.ylabel("Frequency")
plt.savefig("./speed_distribution.png")
plt.close()

env.close()



