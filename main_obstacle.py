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
import rpicam
from initialisation import *
from main import POSITION_FILTER_RATIO, HEADING_FILTER_RATIO, MM_PER_STEPS

outer_one_section = [
# straight path #1
[[200, 1000], [200, 1200]],

[[200, 1200], [200, 1350]],
# check colour
[[200, 1500], [200, 2000]],

# turning point #1
[[200, 2000], [550, 2300]],
# check colour
[[700, 2400], [700, 2450]],
# check colour
]

inner_one_section = [
# straight path #1
[[800, 1000], [800, 1200]],
# check colour
[[800, 1200], [800, 1400]],
# check colour
#turning point #1
[[800, 1400], [800, 2000]],
[[800, 2200], [850, 2200]],
[[850, 2200], [900, 2200]],
# check colour
]

parking_path = [[400, 1450], [400, 1100]]
obstacle_outer_paths, obstacle_inner_paths = full_path_from_one_section(outer_one_section, inner_one_section)
ccw_obstacle_outer_paths, ccw_obstacle_inner_paths = ccw_paths_from_cw(obstacle_outer_paths, obstacle_inner_paths)

def run(gyro, ldr, pi):
    pose = initial_pose() 
    stop_y = pose[1] - 50
    print("Initial pose:", pose)
    print("angle_z =", gyro.angle_z())
    L_dist = min(get_distance(20, ldr), get_distance(25, ldr), get_distance(30, ldr), get_distance(35, ldr), get_distance(40, ldr), get_distance(45, ldr))
    R_dist = min(get_distance(340, ldr), get_distance(335, ldr), get_distance(330, ldr), get_distance(325, ldr), get_distance(320, ldr), get_distance(315, ldr))

    if pose[0] < 1500:
        obstacle_outer_paths = navigation.augment_paths(obstacle_outer_paths)
        obstacle_inner_paths = navigation.augment_paths(obstacle_inner_paths)
        print("clockwise")
    else:
        obstacle_inner_paths = navigation.augment_paths(ccw_obstacle_outer_paths)
        obstacle_outer_paths = navigation.augment_paths(ccw_obstacle_inner_paths)
        print("counter-clockwise")
    paths = obstacle_inner_paths

    index = 0
    print("Paths augmented")
    path_count = 0
    print('steps', drive.steps)
    reset_pose()  
    colour = None

    reverse = False
    if pose[0] < 1500: #left
        if R_dist < 400:
            colour = rpicam.detect_blob()
            if colour == "r":
                reverse = True
                print(colour)
                paths = obstacle_inner_paths
            else:
                print(colour)
                paths = obstacle_outer_paths
    else:
        if L_dist < 400:
            colour = rpicam.detect_blob()
            if colour == "g":
                reverse = True
                print(colour)
                paths = obstacle_outer_paths
            else:
                print(colour)
                paths = obstacle_inner_paths

    while reverse:
        pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)
        if ldr.update():
            # print("working", time.time())
            lidar_measurements = ldr.get_measurements()
            # for l in lidar_measurements:
            #     print(l)
            # print(len(lidar_measurements))
            # break
            #spike detection
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
            
            # if len(matches) == 0:
            #     count += 1
            #     if count > 5:
            #         drive.drive(0)
            #         time.sleep(20)
            #         count = 0
            position_error = calc_position_error(matches)
            spike_pose = calc_pose(pose, position_error)
            # print("Pose after position error:", spike_pose)
            merged_position_pose = merge_positions(pose, spike_pose, POSITION_FILTER_RATIO)
            angle_error = calc_angle_error(merged_position_pose, matches)
            # print(angle_error)
            spike_heading_pose = calc_heading(merged_position_pose, angle_error)
            # print(spike_heading_pose)
            merged_pose = merge_heading(merged_position_pose, spike_heading_pose, HEADING_FILTER_RATIO)
            # print("merged_position_pose:", merged_position_pose[2], "spike_heading_pose:", spike_heading_pose[2])
            # pose = merged_position_pose
            pose = merged_pose

        if pose[0] < 1500: #left
            drive.steer_p_back(90, pose[2], 200)
            if pose[1] <= 1300:
                break
        else: #right
            drive.steer_p_back(90, pose[2], 200)
            if pose[1] <= 1500:
                break 

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
            # print("Pose after position error:", spike_pose)
            merged_position_pose = merge_positions(pose, spike_pose, POSITION_FILTER_RATIO)
            angle_error = calc_angle_error(merged_position_pose, matches)
            # print(angle_error)
            spike_heading_pose = calc_heading(merged_position_pose, angle_error)
            # print(spike_heading_pose)
            merged_pose = merge_heading(merged_position_pose, spike_heading_pose, HEADING_FILTER_RATIO)
            # print("merged_position_pose:", merged_position_pose[2], "spike_heading_pose:", spike_heading_pose[2])
            # pose = merged_position_pose
            pose = merged_pose

        count = navigation.drive_paths(index, paths, pose, 250)

        if count != index:
            path_count += 1
            if count % 4 == 1 and colour != None:
                print("skip check\n\n\n")
            else:
                colour = rpicam.detect_blob()
                print("colour", colour)
                if colour == 'r':
                    paths = obstacle_inner_paths
                elif colour == 'g':
                    paths = obstacle_outer_paths
            print('path', paths[count % len(paths)])

        index = count % len(paths)
        if path_count >= 60:
            if pose[1] >= stop_y:
                print("Reached stopping pose")
                break
        
        with open("troubleshootingData.txt", "w") as f:
            f.write(str(matches), str(merged_position_pose), str(merged_pose))
        
        if pi.read(17) == 0:
            break
    #code here
    while True:
        if pose[0] != parking_path[0][0] or pose[1] != parking_path[0][1]:
            navigation.drive_path_back(parking_path, pose, 200)
            navigation.drive_path(parking_path, pose, 200)
        else:
            break

    drive.drive(0) 
    drive.steering(0)