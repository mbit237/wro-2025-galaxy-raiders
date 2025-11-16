import time
import drive 
import navigation
from initialisation import * 
from gyro import Gyro 
from estimate_pose import estimate_pose, reset_pose
import coind4

MM_PER_STEPS = 0.296
parking_path = [[350, 1100], [350, 1900]]
parking_path = navigation.augment_path(parking_path)
ldr = coind4.CoinD4() #lidar initialise
ldr.start()
print("Lidar started")

gyro = Gyro()
gyro.calibration()
print("Gyro calibrated")
# pose = [640, 1250, 90]



# fwd_stop_y = parking_path[1][1] #1450
# rear_stop_y = parking_path[0][1] #1100
# x_min = parking_path[0][0] - 15
# x_max = parking_path[0][0] + 15
# y_min = 1616 - 15
# y_max = 1616 + 15

# prev_time = time.time()
# parking_start_pos_reached = False

# while True:
#     while pose[1] < fwd_stop_y:
#         pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)
#         navigation.drive_path(parking_path, pose, 200)
#         if (time.time() - prev_time) > 0.5:
#             print(pose)
#             prev_time = time.time()
#         if ((x_min < pose[0] < x_max) and (y_min < pose[1] < y_max) and (87 < pose[2] < 93)):
#             parking_start_pos_reached = True
#             break
#     if parking_start_pos_reached:
#         drive.drive(0)
#         drive.steering(0)
#         print("parking starting pos reached")
#         break    

#     while pose[1] > rear_stop_y:
#         pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)
#         navigation.drive_path_back(parking_path, pose, 200)
#         if (time.time() - prev_time) > 0.5:
#             print(pose)
#             prev_time = time.time()
#         if ((x_min < pose[0] < x_max) and (y_min < pose[1] < y_max) and (87 < pose[2] < 93)):
#             parking_start_pos_reached = True
#             break
#     if parking_start_pos_reached:
#         drive.drive(0)
#         drive.steering(0)
#         print("parking starting pos reached")
#         break    

pose = [346.53589045596135, 1788.7217848972355, 89.71008969053936]

# parking 
# part 1 turn left, move back
drive.steering(-45)
drive.drive(-200)

while pose[1] > 1550:
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)

# part 2 straight, move back
drive.steering(0)
drive.drive(-200)

while pose[1] > 1425:
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)

# part 3 turn right, move back
drive.steering(45)
drive.drive(-200)

while pose[1] > 1350:
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)

# part 4 turn left, move fwd 

drive.steering(-45)
drive.drive(0)
start_time = time.time()
while (time.time() - start_time) < 0.5:
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)
drive.drive(150)

while pose[1] < 1400:
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)

drive.steering(45)
drive.drive(0)
start_time = time.time()
while (time.time() - start_time) < 0.5:
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)
drive.drive(-150)

while pose [1] > 1370:
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)

drive.steering(-45)
drive.drive(0)
start_time = time.time()
while (time.time() - start_time) < 0.5:
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)
drive.drive(150)

while pose[1] < 1420:
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)

drive.steering(0)
drive.drive(-150)

while pose[1] > 1380:
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)

drive.steering(0)
drive.drive(0)
    
print("parked")