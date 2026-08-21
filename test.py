# import pybullet as p
# import time
# import pybullet_data
# import time
# import math
# config ={
#     "gravity":-9.8,
#     "startPos":[0,0,0.2],
#     "target_pos":[[5,0,0],[0,5,0],[-3,-3,0],[0,0,0]],
#     "simu_time":10000000000000,
#     "force":50,
#     "velo":10,
#     "tolerance":0.5,
#     "target_pos_id":0,
#     "danger_distance":0.25,
#     "detect_d":10
# }
# def detect(config):
#     car_pos, car_w = p.getBasePositionAndOrientation(car)
#     yaw = p.getEulerFromQuaternion(car_w)[2]
#     pitch = p.getEulerFromQuaternion(car_w)[1]

#     dx = math.cos(pitch) * math.cos(yaw)
#     dy = math.cos(pitch) * math.sin(yaw)
#     dz = math.sin(pitch)
#     start = [car_pos[0]+dx,
#             car_pos[1]+dy,
#             car_pos[2]-dz+0.3]
#     end = (
#         car_pos[0] + dx * config["detect_d"],
#         car_pos[1] + dy * config["detect_d"],
#         car_pos[2] - dz * config["detect_d"]
#     )

#     # p.addUserDebugLine(
#     #     start,
#     #     end,
#     #     lineColorRGB=[1, 0, 0],
#     #     lineWidth=3,
#     #     lifeTime=0.1
#     # )
#     result = p.rayTest(start, end)[0]
#     return result
# def move_to(target_id,config):

#     check = 0
#     car_pos, car_w = p.getBasePositionAndOrientation(car)
#     distance = [config["target_pos"][target_id][0]-car_pos[0],config["target_pos"][target_id][1]-car_pos[1]]
#     target_deg =math.degrees(math.atan2(distance[1],distance[0]))
#     yaw_deg = math.degrees(p.getEulerFromQuaternion(car_w)[2])

#     diff_ang =target_deg-yaw_deg
#     if diff_ang>180: diff_ang-=360
#     if diff_ang<-180: diff_ang+=360
#     direc = 1 if diff_ang>0 else -1

#     fl =0
#     fr =0
#     rl =0
#     rr =0
#     force = config["force"]
#     velo = config["velo"]
#     if abs(yaw_deg-target_deg) >0:
#         fl +=-velo*direc
#         fr +=velo*direc
#         rl +=-velo*direc
#         rr +=velo*direc

#     if math.sqrt(distance[0]**2+distance[1]**2) !=0:
#         fl +=min(math.log(1+math.sqrt(distance[0]**2+distance[1]**2))*10, 10)
#         fr +=min(math.log(1+math.sqrt(distance[0]**2+distance[1]**2))*10, 10)
#         rl +=min(math.log(1+math.sqrt(distance[0]**2+distance[1]**2))*10, 10)
#         rr +=min(math.log(1+math.sqrt(distance[0]**2+distance[1]**2))*10, 10)
#     if abs(distance[0])+abs(distance[1]) <config["tolerance"]:
#         fl =0
#         fr =0
#         rl =0
#         rr =0
#         check = target_id+1

#     p.setJointMotorControl2(car,2,controlMode = p.VELOCITY_CONTROL,targetVelocity=fl, force = force)
#     p.setJointMotorControl2(car,3,controlMode = p.VELOCITY_CONTROL,targetVelocity=fr, force = force)
#     p.setJointMotorControl2(car,4,controlMode = p.VELOCITY_CONTROL,targetVelocity=rl, force = force)
#     p.setJointMotorControl2(car,5,controlMode = p.VELOCITY_CONTROL,targetVelocity=rr, force = force)
#     return check


# physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
# p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
# p.setGravity(0,0,config["gravity"])

# planeId = p.loadURDF("plane.urdf")
# startOrientation = p.getQuaternionFromEuler([0,0,0])
# #車子
# car = p.loadURDF("/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/car/husky/husky.urdf",
#                     config["startPos"], 
#                     startOrientation)
# #機械手臂
# arm = p.loadURDF("/home/kgforsure/Documents/github_workspace_yeah/arm_camera_put_inside_hole/car/franka_panda/panda.urdf",
#                     [0,0,0.5], startOrientation)
# constraint_id = p.createConstraint(
#     parentBodyUniqueId=car,
#     parentLinkIndex=-1,  # -1 通常代表車體基底 (base_link)
#     childBodyUniqueId=arm,
#     childLinkIndex=-1,  # 如果是單一檔案物件通常也是 -1
#     jointType=p.JOINT_FIXED,  # 設為固定關節 (像焊接一樣)
#     jointAxis=[0, 0, 0],
#     parentFramePosition=[0, 0, 0.5],  # 在父物件（車子）身上的固定點位置
#     childFramePosition=[0, 0, 0],  # 在子物件身上的對齊中心點
#     parentFrameOrientation=[0, 0, 0, 1],
#     childFrameOrientation=[0, 0, 0, 1],
# )
# #障礙物 與 目標指示物
# r2d2 = p.loadURDF("cube.urdf",[5,0,1])
# r2d3 = p.loadURDF("cube.urdf",[0,7,1])
# mugs = []
# for i,j in enumerate(config["target_pos"]):
#     mugs.append(p.loadURDF("urdf/mug.urdf",j, startOrientation))
# #迴圈
# p.resetBasePositionAndOrientation(car, config["startPos"], startOrientation)
# check = 0
# for i in range (config["simu_time"]):
#     p.stepSimulation()

#     #障礙物偵測
#     result = detect(config)
#     #結束條件
#     if config["target_pos_id"] >= len(config["target_pos"]):
#         print("All targets reached!")
#         time.sleep(2)
#         break
#     # 目標前進
#     if result[2]*config["detect_d"]>config["danger_distance"]:
#         reached = move_to(config["target_pos_id"], config)
#         if reached:
#             print(f"Reached target {config["target_pos_id"]}! Moving to next.")
#             config["target_pos_id"] += 1
#     else:
#         p.setJointMotorControl2(car,2,controlMode = p.VELOCITY_CONTROL,targetVelocity=0, force = config["force"])
#         p.setJointMotorControl2(car,3,controlMode = p.VELOCITY_CONTROL,targetVelocity=0, force = config["force"])
#         p.setJointMotorControl2(car,4,controlMode = p.VELOCITY_CONTROL,targetVelocity=0, force = config["force"])
#         p.setJointMotorControl2(car,5,controlMode = p.VELOCITY_CONTROL,targetVelocity=0, force = config["force"])



#     time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(car)
# print(cubePos,cubeOrn)
# p.disconnect()

# import datetime
# now = datetime.datetime.now(tz=datetime.timezone(datetime.timedelta(hours=8)))
# print(now.strftime('%Y%m%d_%H%M'))

# # -*- encoding: utf-8 -*-
# #!/bin/python3
# from Crypto.Cipher import AES
# import base64

# plain_text = b""
# with open("./packet2.txt", "r")as f:
#     for encrypted in f.readlines():
#         encrypted = encrypted.replace("\n","")
#         BS = AES.block_size
#         mode = AES.MODE_CBC
#         pad = lambda s: s + (BS-len(s))*b"\0"
#         pad_txt = lambda s: s + (BS - len(s) % BS) * chr(BS - len(s) % BS)
#         unpad = lambda s : s[0:-ord(s[-1])]
#         key = b"TPONEMESH_Kf!xn?"
#         vi = b"1234567890abcdef"
#         cryptor = AES.new(pad(key), mode, vi)
#         plain_text += cryptor.decrypt(bytes.fromhex(encrypted[32:]))
#         plain_text += b'\n'
# with open("./decrypt2.txt", "wb")as f1:
#     f1.write(plain_text)
# import pickle
# import collections

# # 利用 collections.namedtuple 產生的類別，並手動修正其 module 屬性讓它可以被 pickle 序列化
# cf = collections.namedtuple("E",["e"])
# c = cf(123)
# payload = pickle.dumps(print(123))
# print(payload.hex())

def gen_prime(r):
    while True:
        v = (r.next() + r.A) % r.M

        if is_prime(v):
            return v
def next(r1,r2,r3):
    self.seed = self.seed * self.A + self.B
    self.seed = self.seed % self.M
    return self.seed
n = 44979109816375385151192616993734668238566297274190235143253001178909321789685748949783780256179900720054259174083239485774007638771699762776865247911787951100011957147510134470553803417445338768645750190488175178351636416497362113345093186073732514592691500000060106131643200307374932799352104677022953917929
r1 = 7774080469706542028052089782021726541332013160388645474660746006826129083479192904363768225154008589165127947494958152840398642469668392329432951186189121, 
r2 = 5820931323708717278385542913648368340521297441473881748936757981486214113069931647155806153779678327907554684112886909779238302608329052202908488211196889, 
r3 = 12353988392867421544854313180561660683555910980058952603694950929586432387559301935442629498762388392107985249955985825336134383672034582904567227111628321
val: 16576115284449824053753026975805510807703815731131452775055155621325948694170022356135656215373482666170244603555499747477140548412693599398145898682259084712417703414641309845706910504356048304887844603379197135277834205574425451559275869278321148300185864246800047174642052978339095661652949790423820963037
