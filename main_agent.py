'''
用來跑最終地圖的模型
'''
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from main_game_env import stand
import time
if __name__ == "__main__":
    # 1. 建立環境
    env = stand(render_mode="no") # 如果想加速訓練不看畫面，可改為 "direct"
    
    # 2. 檢查環境格式是否符合 Gymnasium 標準
    check_env(env, warn=True)
    print("環境格式檢驗合格！")

    # 3. 初始化 PPO 模型
    # MlpPolicy 代表使用多層感知機 (全連接網路) 處理向量輸入
    model = PPO("MlpPolicy", env, verbose=1, learning_rate=3e-4, n_steps=2048)

    # 4. 開始訓練 (你可以先用少量的 timesteps 測試，例如 10000)
    print("開始訓練 PPO 模型...")
    model.learn(total_timesteps=5000)

    # 5. 儲存模型
    model.save("ppo_door_out_agent")
    print("模型已儲存！")
    env.close()


    #--------- 
    env = stand(render_mode="human")
    obs, info = env.reset()

    
    # 建立 agent
    agent = PPO("MlpPolicy", env, verbose=1, learning_rate=3e-4, n_steps=2048)
    agent.load("./ppo_door_out_agent.zip")
    
    # 跑幾次動畫看看成果
    for episode in range(20):
        state, _ = env.reset()
        done = False
        total_reward = 0
        #it will hardly done, because too hard to reach goal
        while not done: 
            # 測試時直接用 select_action，此時 agent 已經很聰明了
            action, _= agent.predict(obs, deterministic=True)
            next_state, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated
            state = next_state
            total_reward += reward
            
        print(f"Test Episode {episode+1}, Reward: {total_reward:.2f}")
        time.sleep(0.5)
        print("AAAAAAAAAAAAAAAAAAAAAAAAAA")
    env.close()