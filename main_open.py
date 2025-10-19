#!/usr/bin/python
import pigpio
import time 
import struct
import math 
import coind4
from gyro import Gyro 
import drive
import spike
import navigation
from estimate_pose import estimate_pose, reset_pose
import client_raspi as client
from initialisation import *
from main import POSITION_FILTER_RATIO, HEADING_FILTER_RATIO, MM_PER_STEPS


cw_paths = [ 
    [[300, 300], [300, 2350]],
    [[300, 2700], [2350, 2700]],
    [[2700, 2700], [2700, 650]],
    [[2700, 300], [650, 300]]
]
ccw_paths = [ 
    [[2700, 300], [2700, 2350]],
    [[2700, 2700], [650, 2700]],
    [[300, 2700], [300, 650]],
    [[300, 300], [2350, 300]]
]

def run(gyro, ldr, pi):
    pose = initial_pose(ldr) 
    stop_y = pose[1] - 50
    print("Initial pose:", pose)
    print("angle_z =", gyro.angle_z())

    if pose[0] < 1500:
        paths = cw_paths
    else:
        paths = ccw_paths
    paths = navigation.augment_paths(paths)
    index = 0
    print("Paths augmented")

    path_count = 0
    print('steps', drive.steps)
    reset_pose()  
    while True:
        pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS) 
        if ldr.update():
            lidar_measurements = ldr.get_measurements()
            spikes = spike.identify_spikes(lidar_measurements)
            # print(spikes)
            c_spikes = spike.add_cartesian(pose, spikes)
            # print(pose, c_spikes)
            matches = spike.match_landmarks(c_spikes)
            # client.send(matches)
            print(len(matches))
            while pose[2] > 180:
                pose[2] -= 360
            while pose[2] < -180:
                pose[2] += 360
            print(pose, matches)
            
            position_error = calc_position_error(matches)
            spike_pose = calc_pose(pose, position_error)
            print("Pose after position error:", spike_pose)
            merged_position_pose = merge_positions(pose, spike_pose, POSITION_FILTER_RATIO)
            angle_error = calc_angle_error(merged_position_pose, matches)
            # print(angle_error)
            spike_heading_pose = calc_heading(merged_position_pose, angle_error, HEADING_FILTER_RATIO)
            # print(spike_heading_pose)
            merged_pose = merge_heading(merged_position_pose, spike_heading_pose)
            print("merged_position_pose:", merged_position_pose[2], "spike_heading_pose:", spike_heading_pose[2])
            # pose = merged_position_pose
            pose = merged_pose

        count = navigation.drive_paths(index, paths, pose, 250)
        if count != index:
            path_count += 1
        index = count % 4
        if path_count == 12:
            if pose[1] >= stop_y:
                print("Reached stopping pose")
                break
        
        with open("troubleshootingData.txt", "w") as f:
            f.write(str(matches), str(spike_pose), str(merged_position_pose), str(merged_pose))

        if pi.read(17) == 0:
            break

    drive.drive(0)  
    drive.steering(0)  