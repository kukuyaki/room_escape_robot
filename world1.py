import pybullet as p
cin = p.connect(p.SHARED_MEMORY)
if (cin < 0):
    cin = p.connect(p.GUI)
objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("r2d2.urdf", -0.232202,10.321620,0.185274,-0.164367,0.705068,0.671814,-0.156614)]
ob = objects[0]
jointPositions=[ 0.000000, 0.000000, -0.003967, -0.004770, 0.000000, 0.000000, -0.003375, -0.008278, -0.143128, -0.000006, 0.000000, -0.049746, 0.000000, 0.094077, 0.000000 ]
try:
    # 只要視窗還連線著，且沒超過 10000 步，就持續模擬
    for i in range(10000):
        if not p.isConnected(): # 檢查使用者是否按了 X 關閉
            break
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

finally:
    # 關鍵：不論是自然跑完還是你中途按 X 關閉，都會強制執行這裡的程式碼
    if p.isConnected():
        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
        print("\n【最終位置與姿態】:", cubePos, cubeOrn)
        
        save_path = "/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/world1.py"
        p.saveWorld(save_path)
        print(f"【成功】場景已安全儲存至: {save_path}")
        
        p.disconnect()
    else:
        print("\n【注意】視窗已被手動關閉，但因為是在 GUI 關閉後才嘗試 saveWorld，PyBullet 可能因失去伺服器連線而無法成功導出檔案。")