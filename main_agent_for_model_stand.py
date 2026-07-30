'''
用來訓練走路模型的agent 
TODO
'''
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from main_game_env import stand
import time

def test_m(file_name,ep):
    envtest = stand(render_mode="human")
    obs, info = envtest.reset()
    agent = PPO.load(f"./{file_name}.zip", env=envtest,device="cpu")
    
    obs, _ = envtest.reset()
    done = False
    total_reward = 0
    #it will hardly done, because too hard to reach goal
    while not done: 
        # 測試時直接用 select_action，此時 agent 已經很聰明了
        action, _= agent.predict(obs, deterministic=True)
        next_obs, reward, terminated, truncated, _ = envtest.step(action)
        done = terminated or truncated
        obs = next_obs
        total_reward += reward
        
    print(f"Test Episode {ep+1}, Reward: {total_reward:.2f}")
    time.sleep(0.5)
    envtest.close()
if __name__ == "__main__":
    # 1. 建立環境
    env = stand(render_mode="no")
    
    # 2. 檢查環境格式是否符合 Gymnasium 標準
    check_env(env, warn=True)
    print("環境格式檢驗合格！")

    # 3. 初始化 PPO 模型
    # MlpPolicy 代表使用多層感知機 (全連接網路) 處理向量輸入
    model = PPO("MlpPolicy", env, verbose=1, learning_rate=3e-4, n_steps=2048,device="cpu")
    step_record = 5_000
    target_round = 5
    ite = 0
    for ep in range(target_round):
        ite+=1
        model.learn(total_timesteps=step_record)
        file_name = f"./models/ppo_stand_agent_{ite*step_record}"
        model.save(file_name)
    test_m(file_name,ep)
    env.close()


