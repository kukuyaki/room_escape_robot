'''
用來訓練走路模型的agent 
TODO 要去修獎勵函數
'''
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from main_game_env import stand, walk
import time
import os
import argparse

def train_m(env_name):
    os.makedirs("./models", exist_ok=True)
    match env_name:
        case 'stand':
            env = stand(render_mode = "no")
        case 'walk':
            env = walk(render_mode = "no")
        # case 'grab':
        #     env = grab(render_mode = "no")
        # case 'bi':
        #     env = bi(render_mode = "no")
        case _:
            print('task not found')
            return
    check_env(env, warn=True)
    print("環境格式檢驗合格！")
    obs, info = env.reset()

    model = PPO("MlpPolicy", env, verbose=1, learning_rate=3e-4, n_steps=2048,device="cpu")
    step_record = 25_000
    ite = 0
    while 1:
        ite+=1
        model.learn(total_timesteps=step_record,reset_num_timesteps=False)
        file_name = f"./models/ppo_{env_name}_agent_{ite*step_record}"
        model.save(file_name)
    # test_m(file_name,ep)
    env.close()


def test_m(env_name,file_name):

    match env_name:
        case 'stand':
            env = stand(render_mode = "human")
        case 'walk':
            env = walk(render_mode = "human")
        # case 'grab':
        #     env = grab(render_mode = "human")
        # case 'bi':
        #     env = bi(render_mode = "human")
        case _:
            print('task not found')
            return
    check_env(env, warn=True)
    print("環境格式檢驗合格！")
    obs, info = env.reset()
    agent = PPO.load(f"{file_name}", env=env,device="cpu")
    
    done = False
    total_reward = 0
    step = 0
    while not done: 
        step +=1
        action, _= agent.predict(obs, deterministic=True)
        next_obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        obs = next_obs
        total_reward += reward
        if step%1000:
            print(f"{step = }")
            print(f"{total_reward = }")
            print(f"{info["limit"] = }")

    env.close()
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Train or test model.')
    parser.add_argument('gymenv', help='Gymnasium environment i.e. Humanoid-v4',nargs='?',default="Pusher-v5")
    parser.add_argument('-t', '--train',nargs="?", const="1", default=None,)
    parser.add_argument('-s', '--test', metavar='path_to_model')
    args = parser.parse_args()

    env_name = args.gymenv
    train = args.train
    test = args.test
    file_name = args.test




    if train:
        train_m(env_name)
    if test:
        test_m(env_name, file_name)

    


