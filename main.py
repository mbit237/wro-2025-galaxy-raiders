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

MM_PER_STEPS = 0.296

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

def initial_pose():
    # forward + backward dist, left + right dist, gyro_z
    fwd_dist = get_distance(0)
    left_dist = get_distance(270)
    rear_dist = get_distance(180)
    right_dist = get_distance(90)
    
    #robot is on left side
    if get_distance(360-50) > 1000 or get_distance(360-34) > 1200 or get_distance(180+35) > 1500 or get_distance(180+15) > 1000:
        robot_x = (left_dist + (1000 - right_dist)) / 2           
        robot_y = (rear_dist + (3000 - fwd_dist)) / 2
        print("left side")
        return [robot_x, robot_y, gyro.angle_z()]
    else: # robot is on right side
        robot_x = ((left_dist + 2000) + (3000 - right_dist)) / 2
        robot_y = (rear_dist + (3000 - fwd_dist)) / 2
        print("right side")
        return [robot_x, robot_y, gyro.angle_z()]
    
# while True:
#     if ldr.update():
#         break
# print('cleared')
while True: 
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
        print(matches)


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
