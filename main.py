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
    [[500, 500], [500, 2500]], 
    [[500, 2500], [2500, 2500]], 
    [[2500, 2500], [2500, 500]], 
    [[2500, 500], [500, 500]]
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



ldr = coind4.CoinD4() #lidar initialise
ldr.start()
# compass = Compass()
gyro = Gyro() #initialise class
gyro.calibration()

def get_distance(dir):
    while True: 
        if ldr.update():
            lidar_measurements = ldr.get_measurements()
            for m in lidar_measurements:
                if dir+1 > m[0] > dir-1:
                    return m[1]

def initial_pose(ldr_measurements):
    # REFER TO "Lidar measurements" IN ONSHAPE
    # forward + backward dist, left + right dist, gyro_z
    fwd_dist = get_distance(0)
    left_dist = get_distance(270)
    rear_dist = get_distance(180)
    right_dist = get_distance(90)
    # inner spike at 45-85 deg, outer spike at 5-40 deg

    pos1_line = get_distance(360-50) # imaginary line drawn at pos 1
    pos2_line = get_distance(360-34) # imaginary line at pos 2
    pos3_line = get_distance(180+35) # imaginary line at pos 3
    pos4_line = get_distance(180+15) # imaginary line at pos 4

    check_spike = identify_spikes(ldr_measurements)

    if (pos1_line > 1000 or pos2_line > 1300 or pos3_line > 1500 or pos4_line > 1000): 
        # robot is either on left side pos 1-6 or on right side pos 1, 2, 4 or 5
        if (90 > check_spike[0] > 5 and check_spike[1] > 1100) and (175 > check_spike[0] > 100 and check_spike[1] > 1200):
            # robot is on right side, pos R1, R2, R4 or R5
            robot_x = ((left_dist + 2000) + (3000 - right_dist)) / 2 # take average readings of left and right measurements
            robot_y = (rear_dist + (3000 - fwd_dist)) / 2
            print("right side, pos 1, 2, 4 or 5")
            return [robot_x, robot_y, gyro.angle_z()]
        else:
            # robot is on left side, pos L1-L6
            robot_x = (left_dist + (1000 - right_dist)) / 2
            robot_y = (rear_dist + (3000 - fwd_dist)) / 2
            print("left side, pos 1-6")
            return [robot_x, robot_y, gyro.angle_z()]

    else: # robot is on right side, pos R3 or R6
        robot_x = ((left_dist + 2000) + (3000 - right_dist)) / 2
        robot_y = (rear_dist + (3000 - fwd_dist)) / 2
        print("right side, pos 3 or 6")
        return [robot_x, robot_y, gyro.angle_z()]
    
# while True:
#     if ldr.update():
#         break
# print('cleared')
paths = navigation.augment_paths(paths)
index = 0
while True:
    pose = estimate_pose(initial_pose(), gyro.delta_z(), MM_PER_STEPS) 
    if ldr.update():
        lidar_measurements = ldr.get_measurements()
        # for l in lidar_measurements:
        #     print(l)
        # print(len(lidar_measurements))
        # break
        #spike detection
        spikes = spike.identify_spikes(lidar_measurements)
        c_spikes = spike.add_cartesian(pose, spikes)
        matches = spike.match_landmarks(c_spikes)
        navigation.drive_paths(index, paths, pose, 100)
        print(matches)

    index = navigation.drive_paths(index, paths, pose, 100)
    index %= 4

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
