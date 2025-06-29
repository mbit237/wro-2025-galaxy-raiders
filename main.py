import pigpio
import time 
import struct
import math 
# import client_raspi

# import lds02rr
import coind4
# from compass import Compass
from gyro import Gyro 
import drive
import spike
import navigation
from estimate_pose import estimate_pose

MM_PER_STEPS = 0.296

paths = [
    [[500, 500], [500, 2200]], 
    [[500, 2500], [2200, 2500]], 
    [[2500, 2500], [2500, 800]], 
    [[2500, 500], [800, 500]]
]

# pose = [600, 1600, 90] # Initial pose (mm)



# Test motor encoder
# drive.drive(155)
# time.sleep(3)
# drive.drive(0)
# print(drive.steps)
# drive.drive(-155)
# time.sleep(3)
# drive.drive(0)
# print(drive.steps)

def get_distance(dir):
    while True: 
        if ldr.update():
            lidar_measurements = ldr.get_measurements()
            for m in lidar_measurements:
                if dir+1 > m[0] > dir-1:
                    return m[1]

def identify_closer_spikes(measurements):
    closer_spikes = []
    for d in range(len(measurements)):
        prev_d = measurements[d-1][1]
        curr_d = measurements[d][1]
        next_d = measurements[(d+1) % len(measurements)][1] 
        #check if the difference between the previous and the current is signifcantly smaller/bigger than the difference between current and next
        if (prev_d - curr_d > 300 or next_d - curr_d > 300) and (curr_d > 200):
                closer_spikes.append(measurements[d])
    
    return closer_spikes


def initial_pose():
    # forward + backward dist, left + right dist, gyro_z
    fwd_dist = get_distance(0)
    left_dist = get_distance(90)
    rear_dist = get_distance(180)
    right_dist = get_distance(270)

    #robot is on left side
    vote = 0
    position = 0
    while True:
        if ldr.update():
            identified_spikes = identify_closer_spikes(ldr.get_measurements())
            for closer_spike in identified_spikes:
                if 0 < closer_spike[0] < 180:
                    vote += 1
                if 180 < closer_spike[0] < 360:
                    vote -= 1
            if vote <= -10:
                x = (left_dist + (1000 - right_dist)) / 2
                y = ((3000 - fwd_dist) + rear_dist) / 2
                print("left side", "x:", x, "y:", y)
                return [x, y, 90]
            if vote >= 10:
                x = ((left_dist + (1000 - right_dist)) / 2) + 2000
                y = ((3000 - fwd_dist) + rear_dist) / 2
                print("right side", "x:", x, "y:", y)
                return [x, y, 90]
    # if get_distance(360-50) > 1000 or get_distance(360-34) > 1200 or get_distance(180+35) > 1500 or get_distance(180+15) > 1000:
    #     robot_x = (left_dist + (1000 - right_dist)) / 2           
    #     robot_y = (rear_dist + (3000 - fwd_dist)) / 2
    #     print("left side")
    #     return [robot_x, robot_y, gyro.angle_z()]
    # else: # robot is on right side
    #     robot_x = ((left_dist + 2000) + (3000 - right_dist)) / 2
    #     robot_y = (rear_dist + (3000 - fwd_dist)) / 2
    #     print("right side")
    #     return [robot_x, robot_y, gyro.angle_z()]
    
# while True:
#     if ldr.update():
#         break
# print('cleared')
ldr = coind4.CoinD4() #lidar initialise
ldr.start()
print("Lidar started")
# compass = Compass()
gyro = Gyro() #initialise class
gyro.calibration()
print("Gyro calibrated")

paths = navigation.augment_paths(paths)
index = 0
print("Paths augmented")
# print('start')

# while True:
#     initial_pose()
#     print('done')
    # pose = estimate_pose(initial_pose(), gyro.delta_z(), MM_PER_STEPS)
    # if ldr.update():
    #     print("working")
    #     lidar_measurements = ldr.get_measurements()
    #     closer_spikes = identify_closer_spikes(lidar_measurements)
    #     print(closer_spikes)

pose = initial_pose()
print("Initial pose:", pose)
print("angle_z =", gyro.angle_z())
time.sleep(2)
while True:
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS) 
    # print("Pose:", pose)
    if ldr.update():
        print("working")
        lidar_measurements = ldr.get_measurements()
        # for l in lidar_measurements:
        #     print(l)
        # print(len(lidar_measurements))
        # break
        #spike detection
        spikes = spike.identify_spikes(lidar_measurements)
        closer_spikes = identify_closer_spikes(lidar_measurements)
        print(closer_spikes)
        c_spikes = spike.add_cartesian(pose, spikes)
        matches = spike.match_landmarks(c_spikes)
        print(matches)

    index = navigation.drive_paths(index, paths, pose, 250)
    index %= 4
    # if index == 1:
    #     break

drive.drive(0)  # Stop the robot by setting speed to 0
# print_time = time.time() + 2
# stop_time = time.time() + 10
# while True:
#     ldr.update()

#     if time.time() > print_time:
#         print(ldr.get_rpm())
#         print(ldr.get_measurements())
#         print_time = time.time() + 0.5

    # gyro.update_angle()
    # if time.time() > print_time:
    #     print(gyro.angle_z())
    #     print(drive.steps)
    #     print_time = time.time() + 0.5 
    # drive.steer_p(0, gyro.angle_z(), 200)

    # if time.time() > stop_time:
    #     break
    

## -------- Finding min and max compass angle -------- ##
# if d[0] > lx:
#     lx = d[0]
# elif d[0] < mx:
#     mx = d[0]
# if d[2] > ly:
#     ly = d[2]
# elif d[2] < my:
#     my = d[2]


# Close handle 
# pi.close() -- check 
