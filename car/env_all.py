'''
等手臂訓練完畢後，就要來整合全部功能
場景：目標物、讀卡機、門
初始化：手臂收起來、車子生成在(0,0)、卡片生在5*5的隨機範圍，讀卡機生成在牆壁上、門在固定位置、四周有牆壁
流程：
    判斷目標物位置，指派車子走到目標物周圍可讓機械手臂抓取的範圍
    機械手臂抓取目標後，要收縮回去至穩定位置
    車子走到讀卡機周圍，手臂伸縮至讀卡機，一樣收縮回穩定位置
    車子開出門口
'''
import numpy as np
import pybullet as p
from stable_baselines3 import PPO
import time
import pybullet_data
import time
import math
config ={
    "gravity":-9.8,
    "startPos":[0,0,0.2],
    "target_pos":[[5,0,0],[0,5,0],[-3,-3,0],[0,0,0]],
    "simu_time":10000000000000,
    "force":85,
    "velo":10,
    "tolerance":0.8, #車與目標物的距離
    "target_pos_id":0,
    "danger_distance":0.25,
    "detect_d":10,
    "turn_var":0.5,
    "max_arm_steps":1000
}
model = PPO.load("./car/models/car_grap_observation_v2")


def detect(config):
    car_pos, car_w = p.getBasePositionAndOrientation(car)
    yaw = p.getEulerFromQuaternion(car_w)[2]
    pitch = p.getEulerFromQuaternion(car_w)[1]

    dx = math.cos(pitch) * math.cos(yaw)
    dy = math.cos(pitch) * math.sin(yaw)
    dz = math.sin(pitch)
    start = [car_pos[0]+dx,
            car_pos[1]+dy,
            car_pos[2]-dz+0.3]
    end = (
        car_pos[0] + dx * config["detect_d"],
        car_pos[1] + dy * config["detect_d"],
        car_pos[2] - dz * config["detect_d"]
    )

    # p.addUserDebugLine(
    #     start,
    #     end,
    #     lineColorRGB=[1, 0, 0],
    #     lineWidth=3,
    #     lifeTime=0.1
    # )
    result = p.rayTest(start, end)[0]
    return result
def move_to(target_id,config):

    check = 0
    car_pos, car_w = p.getBasePositionAndOrientation(car)
    target_x = config["target_pos"][target_id][0]
    target_y = config["target_pos"][target_id][1]
    distance = [target_x - car_pos[0],   target_y - car_pos[1]]
    target_deg =math.degrees(math.atan2(distance[1],distance[0]))
    yaw_deg = math.degrees(p.getEulerFromQuaternion(car_w)[2])


    diff_ang =target_deg-yaw_deg
    if diff_ang>180: diff_ang-=360
    if diff_ang<-180: diff_ang+=360
    direc = 1 if diff_ang>0 else -1

    fl =0
    fr =0
    rl =0
    rr =0
    force = config["force"]
    velo = config["velo"]
    if abs(yaw_deg-target_deg) >0:
        fl +=-velo*direc*config["turn_var"]
        fr +=velo*direc*config["turn_var"]
        rl +=-velo*direc*config["turn_var"]
        rr +=velo*direc*config["turn_var"]

    if math.sqrt(distance[0]**2+distance[1]**2) !=0:
        fl +=min(math.log(1+math.sqrt(distance[0]**2+distance[1]**2))*10, 10)
        fr +=min(math.log(1+math.sqrt(distance[0]**2+distance[1]**2))*10, 10)
        rl +=min(math.log(1+math.sqrt(distance[0]**2+distance[1]**2))*10, 10)
        rr +=min(math.log(1+math.sqrt(distance[0]**2+distance[1]**2))*10, 10)
    if math.sqrt(distance[0]**2 + distance[1]**2)<config["tolerance"]:
        print("inside tolerance")
        fl =0
        fr =0
        rl =0
        rr =0
        check = 1

    p.setJointMotorControl2(car,2,controlMode = p.VELOCITY_CONTROL,targetVelocity=fl, force = force)
    p.setJointMotorControl2(car,3,controlMode = p.VELOCITY_CONTROL,targetVelocity=fr, force = force)
    p.setJointMotorControl2(car,4,controlMode = p.VELOCITY_CONTROL,targetVelocity=rl, force = force)
    p.setJointMotorControl2(car,5,controlMode = p.VELOCITY_CONTROL,targetVelocity=rr, force = force)
    return check


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,config["gravity"])

planeId = p.loadURDF("plane.urdf")
startOrientation = p.getQuaternionFromEuler([0,0,0])
#車子
car = p.loadURDF("/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/car/husky/husky.urdf",
                    config["startPos"], 
                    startOrientation)
#機械手臂
arm = p.loadURDF("/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/car/franka_panda/panda.urdf",
                    [0,0,0.5], startOrientation)
constraint_id = p.createConstraint(
    parentBodyUniqueId=car,
    parentLinkIndex=-1,  # -1 通常代表車體基底 (base_link)
    childBodyUniqueId=arm,
    childLinkIndex=-1,  # 如果是單一檔案物件通常也是 -1
    jointType=p.JOINT_FIXED,  # 設為固定關節 (像焊接一樣)
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0.5],  # 在父物件（車子）身上的固定點位置
    childFramePosition=[0, 0, 0],  # 在子物件身上的對齊中心點
    parentFrameOrientation=[0, 0, 0, 1],
    childFrameOrientation=[0, 0, 0, 1],
)
#障礙物 與 目標指示物
# r2d2 = p.loadURDF("cube.urdf",[5,0,1])
# r2d3 = p.loadURDF("cube.urdf",[0,7,1])
mugs = []
for i,j in enumerate(config["target_pos"]):
    mugs.append(p.loadURDF("urdf/mug.urdf",j, startOrientation))

arm_step_counter = 0

card_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
card_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=[0.8, 0.2, 0.2, 1]) # 紅色
cardId = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=card_col, baseVisualShapeIndex=card_vis, basePosition=[0,5,0.8])

#迴圈
robot_state = "Arm_Grasp" #"Arm_Grasp", "Arm_bi"
p.resetBasePositionAndOrientation(car, config["startPos"], startOrientation)
check = 0
for i in range (config["simu_time"]):
    p.stepSimulation()
    if config["target_pos_id"] >= len(config["target_pos"]):
        print("All targets reached!")
        time.sleep(2)
        break
    #模式
    if robot_state == "Navigating":
        #障礙物偵測
        result = detect(config)
        # 目標前進
        if result[2]*config["detect_d"]>config["danger_distance"]:
            reached = move_to(config["target_pos_id"], config)
            if reached:
                config["target_pos_id"] += 1
                print(f"Reached target {config['target_pos_id']}! Stopping car to start arm grasping.")
                # 煞車停下車子
                for wheel in [2, 3, 4, 5]:
                    p.setJointMotorControl2(car, wheel, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=config["force"])
                # 切換狀態到手臂抓取
                robot_state = "Arm_Grasp"   #TODO
                # robot_state = "Arm_bi"   #TODO
                time.sleep(3)
                arm_step_counter = 0
        else:
            for wheel in [2, 3, 4, 5]:
                p.setJointMotorControl2(car, wheel, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=config["force"])

    if robot_state == "Arm_Grasp":
        ########################################################################
        card_pos, _ = p.getBasePositionAndOrientation(cardId)   #TODO
        grap_pos = p.getLinkState(arm,11)[0]
        joint_position = []
        joint_velocity = []


        for i in [0,1,2,3,4,5,6,9]:
            joint_state = p.getJointState(arm, i)
            joint_position.append(joint_state[0]) # 該關節當前的角度 (position)
            joint_velocity.append(joint_state[1]) # 該關節當前的速度 (velocity)

        relative_pos = np.array(card_pos) - np.array(grap_pos)

        obs = np.concatenate([
            joint_position,
            joint_velocity,
            relative_pos,
            card_pos
        ]).astype(np.float32)
        ########################################################################
        action, _states = model.predict(obs, deterministic=True)
        action_joinID_maxV = [
            [0,   0,   2.175, -0.36],
            [1,   1,   2.175, -1.59],
            [2,   2,   2.175, -0.11],
            [3,   3,   2.175, -3.14],
            [4,   4,   2.61,   2.97],
            [5,   5,   2.61,   0.99],
            [6,   6,   2.61,   1.07],
            [7,   9,   0.2,    0.04],
        ]
        for a,j,v,_ in action_joinID_maxV:
            target_vel = float(action[a]) * v * 5 # 放大係數可依訓練速需求調整
            p.setJointMotorControl2(
                bodyUniqueId=arm,
                jointIndex=j,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=target_vel,
                force=50
            )
        print(arm_step_counter)
        arm_step_counter += 1

        # 4. 判斷手臂是否完成抓取，或超時
        # 這裡可以用「超過固定步數」或「夾爪成功夾到物件」來作為結束條件
        if arm_step_counter >= config["max_arm_steps"]:
            print(f"Arm finished grasping for target {config['target_pos_id']}. Resuming navigation.")
            robot_state = "Navigating"  # 切回車子導航去下一個點
            #arm go back stable pos
            for a,j,v,stable_pos in action_joinID_maxV[:-1]:
                target_pos = stable_pos # 放大係數可依訓練速需求調整
                p.setJointMotorControl2(
                    bodyUniqueId=arm,
                    jointIndex=j,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_pos,
                    force=50
                )
        pass
    time.sleep(1./240.)

p.disconnect()