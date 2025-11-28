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

POSITION_FILTER_RATIO = 0.1 
HEADING_FILTER_RATIO = 0.01  
MM_PER_STEPS = 0.296

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
    # client.connect()
    while ldr.update():
        pass
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
    data_send_server = []
    while True:
        pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS) 
        if ldr.update():
            lidar_measurements = ldr.get_measurements()
            spikes = spike.identify_spikes(lidar_measurements)
            # print(spikes)
            c_spikes = spike.add_cartesian(pose, spikes)
            # print(pose, c_spikes)
            matches = spike.match_landmarks(c_spikes)
            
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
            spike_heading_pose = calc_heading(merged_position_pose, angle_error)
            # print(spike_heading_pose)
            merged_pose = merge_heading(merged_position_pose, spike_heading_pose, HEADING_FILTER_RATIO)
            print("merged_position_pose:", merged_position_pose[2], "spike_heading_pose:", spike_heading_pose[2])
            # pose = merged_position_pose
            pose = merged_pose
            data_send_server = [spike_pose, merged_pose, matches, None, [angle_error, position_error]]
            with open("troubleshootingData.txt", "w") as f:
                troubleshoot_data = str(matches) + ", " + str(merged_position_pose) + ", " + str(merged_pose)
                f.write(troubleshoot_data)
                
        count = navigation.drive_paths(index, paths, pose, 250)
        if count != index:
            path_count += 1
        index = count % 4
        if data_send_server:
            data_send_server[3] = index
            # client.send(data_send_server)
            data_send_server = []

        if path_count == 12:
            if pose[1] >= stop_y:
                print("Reached stopping pose")
                break
        
        
        if pi.read(17) == 0:
            break
    # client.send(client.DISCONNECT_MESSAGE)
    drive.drive(0)  
    drive.steering(0)  