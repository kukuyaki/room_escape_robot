import sys
import os

# 如果你的訓練或測試腳本在 stand 子資料夾內，記得把根目錄加進來（根據你先前的結構）
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import gymnasium as gym

# 1. 確保你有引入定義環境的檔案，這樣才會執行到 `gym.register`
gym.register(
    id="stand-v0",
    entry_point="stand.stand_env:stand", 
)
try:
    # 2. 嘗試透過註冊的 ID 建立環境
    env = gym.make("stand-v0", render_mode="no")
    print("✅ 成功建立環境 (gym.make 正常運作)！")

    # 3. 測試 reset
    obs, info = env.reset()
    print(f"✅ Reset 成功！觀察值形狀 (Shape): {obs.shape}")
    print(f"✅ 初始 Info: {info}")

    # 4. 測試 step
    action = env.action_space.sample()  # 隨機產生一個合法動作
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"✅ Step 成功！獲得獎勵: {reward}, Terminated: {terminated}")

    env.close()
    print("🎉 恭喜！你的環境註冊與基本介面完全正常！")

except Exception as e:
    print("❌ 測試失敗，錯誤訊息如下：")
    import traceback
    traceback.print_exc()