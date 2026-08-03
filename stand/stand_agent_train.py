'''
用來訓練走路模型的agent 
TODO 要去修獎勵函數
uv run main_agent.py stand -t ./models/ppo......
uv run main_agent.py stand -s ./models/ppo......
'''
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import time
import os
import gymnasium as gym
import argparse
import torch
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize, VecVideoRecorder
import sys

class EpisodeCounterWrapper(gym.Wrapper):
    def __init__(self, env):
        super().__init__(env)
        self.episode_counter = 0

    def reset(self, **kwargs):
        self.episode_counter += 1
        if kwargs.get("options") is None:
            kwargs["options"] = {}
        # 將全域的 episode 數塞進 options
        kwargs["options"]["episode_id"] = self.episode_counter
        return super().reset(**kwargs)


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

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
gym.register(
    id="stand-v0",
    entry_point="stand.stand_env:stand", 
)


def train_m(file_name,device_name):
    os.makedirs("./stand/models", exist_ok=True)
    os.makedirs("./stand/pkls", exist_ok=True)



    env = make_vec_env(
        env_id="stand-v0",
        n_envs=config["n_envs"],
        wrapper_class=EpisodeCounterWrapper,
        env_kwargs={
            "render_mode": None,    # 训练时不渲染
            # "exclude_current_positions_from_observation": True,
            # "reset_noise_scale": 0.2  # 初始状态噪声
        }
    )

    env = VecNormalize(env, norm_obs=True, norm_reward=True)

    obs= env.reset()

    model = PPO(
        policy="MlpPolicy",  # 使用内置的 MlpPolicy
        env=env,
        device=device_name,
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
    train_before = 0

    try:
        model.learn(
            total_timesteps=config["total_timesteps"],
            progress_bar=True,
            tb_log_name=f"PPO_{device_name.upper()}",
            reset_num_timesteps=False
        )
    finally:
        # 保存模型和归一化参数
        model.save("./stand/models/humanoid_stand_ppo")
        env.save("./stand/pkls/humanoid_vecnormalize.pkl")
        env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Train or test model.')
    parser.add_argument('gymenv', help='Gymnasium environment i.e. Humanoid-v4',nargs='?',default="Pusher-v5")
    parser.add_argument('-t', '--train',nargs="?", const="1", default=None,)
    args = parser.parse_args()

    env_name = args.gymenv
    train_file = args.train

    device_name = "cpu"
    train_m(train_file,device_name)

    


