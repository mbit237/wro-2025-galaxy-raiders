#!/usr/bin/python
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
from estimate_pose import estimate_pose, reset_pose
import client_raspi as client

ldr = coind4.CoinD4() #lidar initialise
ldr.start()

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
    while True:
        if ldr.update():
            identified_spikes = identify_closer_spikes(ldr.get_measurements())
            for closer_spike in identified_spikes:
                if 0 < closer_spike[0] < 180:
                    vote += 1
                if 180 < closer_spike[0] < 360:
                    vote -= 1
            if vote <= -10: # left side
                x = (left_dist)
                y = ((3000 - fwd_dist) + rear_dist) / 2
                print("left side", "x:", x, "y:", y, "fwd_dist:", fwd_dist, "rear_dist:", rear_dist)
                return [x, y, 90]
            if vote >= 10: # right side
                x = ((1000 - right_dist)) + 2000
                y = ((3000 - fwd_dist) + rear_dist) / 2
                print("right side", "x:", x, "y:", y, "fwd_dist:", fwd_dist, "rear_dist:", rear_dist)
                return [x, y, 90]
while True:
    fwd_dist = get_distance(0)
    left_dist = get_distance(90)
    rear_dist = get_distance(180)
    right_dist = get_distance(270)
    print('fwd:', fwd_dist, 'rear:', rear_dist) 
    time.sleep(1)