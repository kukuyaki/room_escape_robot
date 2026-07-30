import pybullet as p
import pybullet_data
import time
import random
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

human = p.loadURDF("/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/humanoid3_colli.urdf",[0,0,1])
joint_NameToId = {}
num_joints = p.getNumJoints(human)
for i in range(num_joints):
    info = p.getJointInfo(human, i)
    joint_name = info[1].decode('utf-8')
    joint_NameToId[joint_name] = i

room_size = 5
wall_col1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, room_size, 2])
wall_col2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[room_size, 0.2, 2])
wall_col3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, (room_size/2)-0.25, 2])

wallId_1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col1, basePosition=[-room_size, 0, 0])
wallId_2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col2, basePosition=[0, -room_size, 0])
offf = (room_size/2)+0.25
wallId_3_L = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col3, basePosition=[room_size,offf, 0])
wallId_3_R = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col3, basePosition=[room_size,-offf, 0])
wallId_4 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col2, basePosition=[0, room_size, 0])

direction_wall = random.choice([room_size-0.2,-room_size+0.2])
wall_size_radom = random.uniform(-room_size+0.3,room_size-0.3)
avoid_door = wall_size_radom if abs(wall_size_radom) > 1 else 1.5

respon_area_reader_v = [wall_size_radom,direction_wall, 1]
respon_area_reader_h = [direction_wall,avoid_door, 1]
reader_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])
reader_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2], rgbaColor=[0.2, 0.2, 0.8, 1]) # 藍色
readerId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=reader_col, baseVisualShapeIndex=reader_vis, basePosition=random.choice([respon_area_reader_v,respon_area_reader_h]))

respon_area_card = [random.uniform(-room_size+1,room_size-1), random.uniform(-room_size+1,room_size-1), 0.5]
card_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.02])
card_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.02], rgbaColor=[0.8, 0.2, 0.2, 1]) # 紅色
cardId = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=card_col, baseVisualShapeIndex=card_vis, basePosition=respon_area_card)


door_link_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.5, 1.0])
door_link_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.5, 1.0], rgbaColor=[0.5, 0.5, 0.5, 1]) # 灰色


doorId = p.createMultiBody(
    baseMass=0,                        # 基座固定
    baseCollisionShapeIndex=-1,        # 基座沒有形狀
    baseVisualShapeIndex=-1,           # 基座沒有外觀
    basePosition=[room_size-0.22, 0, 1],        # 整體的起始位置
    
    linkMasses=[10.0],
    linkCollisionShapeIndices=[door_link_col],
    linkVisualShapeIndices=[door_link_vis],
    linkPositions=[[0, 0, 0]],                  # 相對基座的位置
    linkOrientations=[[0, 0, 0, 1]],            # 相對基座的姿態
    linkInertialFramePositions=[[0, 0, 0]],     # 慣性中心位置
    linkInertialFrameOrientations=[[0, 0, 0, 1]],# 慣性中心姿態
    linkParentIndices=[0],                      # 父節點索引 (0 代表連接到基座)
    linkJointTypes=[p.JOINT_PRISMATIC],         # 關鍵：滑軌關節
    linkJointAxis=[[0, 0, 1]]                     # 沿著 Y 軸左右移動
)

TRIGGER_DISTANCE = 0.5
door_opened = False

print("模擬開始")
door_timer = 0

pick_constraint = None

while p.isConnected():

    closest_points = p.getClosestPoints(cardId, readerId, 1.0)
    if closest_points:
        # closest_points[0][8] 代表兩個物體之間的最短距離 (Distance)
        current_distance = closest_points[0][8]
        if current_distance < TRIGGER_DISTANCE:
            door_timer = 20 * 240  #20秒
    if door_timer != 0:
        door_timer -= 1
        p.setJointMotorControl2(bodyUniqueId=doorId, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=5.0, force=500)
    else:
        p.setJointMotorControl2(bodyUniqueId=doorId, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=0.0, force=500)


    closest_points_card_rh= p.getClosestPoints(
        bodyA=cardId, 
        bodyB=human, 
        distance=1.0, 
        linkIndexB=29  # 指定 human 的哪一個 link
    )
    closest_points_card_lh = p.getClosestPoints(
        bodyA=cardId, 
        bodyB=human, 
        distance=1.0, 
        linkIndexB=22  # 指定 human 的哪一個 link
    )


    contact_points = p.getContactPoints(bodyA=human, bodyB=cardId,linkIndexA=29)
    if contact_points:
        rh_state = p.getLinkState(human, 29)
        rh_pos = rh_state[0]       # (x, y, z)
        rh_orn = rh_state[1]       # (x, y, z, w) 四元數
        
        card_state = p.getBasePositionAndOrientation(cardId)
        card_pos = card_state[0]
        card_orn = card_state[1]
        
        inv_rh_pos, inv_rh_orn = p.invertTransform(rh_pos, rh_orn)
        rel_pos, rel_orn = p.multiplyTransforms(inv_rh_pos, inv_rh_orn, card_pos, card_orn)
        
        pick_constraint = p.createConstraint(
            parentBodyUniqueId=human,
            parentLinkIndex=29,         # 綁定在右手 (link 29)
            childBodyUniqueId=cardId,   # 目標是卡片
            childLinkIndex=-1,          # -1 代表卡片的 base
            jointType=p.JOINT_FIXED,    # 固定約束（相對位置與角度永遠不變）
            jointAxis=[0, 0, 0],
            parentFramePosition=rel_pos,# 剛剛計算好的相對位置
            childFramePosition=[0, 0, 0],
            parentFrameOrientation=rel_orn, # 相對旋轉
            childFrameOrientation=[0, 0, 0]
        )
        p.setCollisionFilterPair(human, cardId, -1, -1, 0)
    p.setJointMotorControl2(
                bodyUniqueId=human,
                jointIndex=20,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=float(2),
                force=10 
            )
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
    human_pos = p.getBasePositionAndOrientation(human)
    
    if human_pos[0][0] >5 :
        print("finish!!")
        for i in range(3*240):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
        break
