
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize, VecVideoRecorder
import torch

# 硬件加速配置
device = "cuda" if torch.cuda.is_available() else "cpu"
torch.backends.cudnn.benchmark = True  # 启用CUDA加速

# 超参数配置
config = {
    "n_envs": 8,               # 并行环境数量（建议设置为CPU核心数）
    "total_timesteps": 5_000_000,  # 总训练步数（至少需要500万步）
    #預設為net_arch=[dict(pi=[64, 64], vf=[64, 64])]、啟動函數 activation_fn=nn.Tanh，以及特徵提取器 features_extractor_class=FlattenExtractor
    "policy_kwargs": {
        "net_arch": {
            "pi": [512, 512],  # 策略网络结构
            "vf": [512, 512]   # 值函数网络结构
        },
        "activation_fn": torch.nn.ReLU
    },
    "learning_rate": 3e-4,
    "batch_size": 4096,         # 匹配GPU显存容量 預設為64
    "n_steps": 2048,            # 每环境采样步数
    "gamma": 0.99,              # 折扣因子
    "gae_lambda": 0.95,
    "clip_range": 0.2,
    "ent_coef": 0.001,          # 适度鼓励探索 預設為0.0
    "target_kl": 0.05,          # KL散度阈值 預設為None
    "max_grad_norm": 0.5        # 梯度裁剪
}

# 创建并行环境（带观察归一化）
env = make_vec_env(
    env_id="Humanoid-v5",
    n_envs=config["n_envs"],
    env_kwargs={
        "render_mode": None,    # 训练时不渲染
        "exclude_current_positions_from_observation": True,
        "reset_noise_scale": 0.2  # 初始状态噪声
    }
)
env = VecNormalize(env, norm_obs=True, norm_reward=True)

# 可选：训练过程视频记录（每10万步保存一次）
if config["n_envs"] == 1:  # 视频录制需要单环境
    env = VecVideoRecorder(
        env,
        video_folder="./videos/",
        record_video_trigger=lambda x: x % 100000 == 0,
        video_length=200
    )

# 初始化PPO模型
model = PPO(
    policy="MlpPolicy",  # 使用内置的 MlpPolicy
    env=env,
    device=device,
    verbose=1,
    tensorboard_log="./tb_logs/humanoid_walk/",
    policy_kwargs=config["policy_kwargs"],  # 传递策略网络参数
    learning_rate=config["learning_rate"],
    batch_size=config["batch_size"],
    n_steps=config["n_steps"],
    gamma=config["gamma"],
    gae_lambda=config["gae_lambda"],
    clip_range=config["clip_range"],
    ent_coef=config["ent_coef"],
    target_kl=config["target_kl"],
    max_grad_norm=config["max_grad_norm"]
)

# 训练循环
try:
    model.learn(
        total_timesteps=config["total_timesteps"],
        progress_bar=True,
        tb_log_name=f"PPO_{device.upper()}",
        reset_num_timesteps=False
    )
finally:
    # 保存模型和归一化参数
    model.save("humanoid_walk_ppo")
    env.save("humanoid_vecnormalize.pkl")
    env.close()