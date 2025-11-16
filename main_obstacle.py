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

POSITION_FILTER_RATIO = 0.1 
HEADING_FILTER_RATIO = 0.01  
MM_PER_STEPS = 0.296

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

parking_path = [[400, 1100], [400, 1900]]

obstacle_outer_paths, obstacle_inner_paths = full_path_from_one_section(outer_one_section, inner_one_section)
ccw_obstacle_outer_paths, ccw_obstacle_inner_paths = ccw_paths_from_cw(obstacle_outer_paths, obstacle_inner_paths)

def park(pose, gyro):
    global parking_path
    print("starting parking")
    fwd_stop_y = parking_path[1][1] #1450
    rear_stop_y = parking_path[0][1] #1100
    x_min = parking_path[0][0] - 15
    x_max = parking_path[0][0] + 15
    y_min = 1616 - 15
    y_max = 1616 + 15

    print(pose)#1278
    prev_time = time.time()
    parking_start_pos_reached = False

    while True:
        while pose[1] < fwd_stop_y:
            pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)
            navigation.drive_path(parking_path, pose, 200)
            if (time.time() - prev_time) > 0.5:
                print(pose)
                prev_time = time.time()
            if ((x_min < pose[0] < x_max) and (y_min < pose[1] < y_max) and (87 < pose[2] < 93)):
                parking_start_pos_reached = True
                break
        if parking_start_pos_reached:
            drive.drive(0)
            drive.steering(0)
            print("parking starting pos reached")
            break    

        while pose[1] > rear_stop_y:
            pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)
            navigation.drive_path_back(parking_path, pose, 200)
            if (time.time() - prev_time) > 0.5:
                print(pose)
                prev_time = time.time()
            if ((x_min < pose[0] < x_max) and (y_min < pose[1] < y_max) and (87 < pose[2] < 93)):
                parking_start_pos_reached = True
                break
        if parking_start_pos_reached:
            drive.drive(0)
            drive.steering(0)
            print("parking starting pos reached")
            break    

    # parking 
    # part 1 turn left, move back
    drive.steering(-45)
    drive.drive(-200)
    
    while pose[1] > 1350:
        pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)
        pass

    
    while pose[1] > 4456:
        estimate_pose()
        drive.steer_p_back(-135, pose[2], 200)


    # part 2 straight, move back


    # part 3 turn right, move back


    drive.steer_p_back(-135, 0, 200)
    while pose[1] > 1350: # tested 
        drive.drive(-200)
    drive.steer_p_back(0, -135, 200)
    while pose[1] > 1300:
        drive.drive(-200)
    print('parked')

def run(gyro, ldr, pi):
    global parking_path, obstacle_inner_paths, obstacle_outer_paths, ccw_obstacle_inner_paths, ccw_obstacle_outer_paths
    client.connect()
    spike_pose = None
    merged_pose = None
    matches = None
    pose = initial_pose(ldr) 
    stop_y = pose[1] - 50
    print("Initial pose:", pose)
    print("angle_z =", gyro.angle_z())
    L_dist = min(get_distance(ldr, 20), get_distance(ldr, 25), get_distance(ldr, 30), get_distance(ldr, 35), get_distance(ldr, 40), get_distance(ldr, 45))
    R_dist = min(get_distance(ldr, 340), get_distance(ldr, 335), get_distance(ldr, 330), get_distance(ldr, 325), get_distance(ldr, 320), get_distance(ldr, 315))

    if pose[0] < 1500:
        obstacle_outer_paths = navigation.augment_paths(obstacle_outer_paths)
        obstacle_inner_paths = navigation.augment_paths(obstacle_inner_paths)
        print("clockwise")
    else:
        obstacle_inner_paths = navigation.augment_paths(ccw_obstacle_outer_paths)
        obstacle_outer_paths = navigation.augment_paths(ccw_obstacle_inner_paths)
        print("counter-clockwise")
    parking_path = navigation.augment_path(parking_path)
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
            data_send_server = [spike_pose, merged_pose, matches]
            with open("troubleshootingData.txt", "w") as f:
                troubleshoot_data = str(matches) + ", " + str(merged_position_pose) + ", " + str(merged_pose)
                f.write(troubleshoot_data)
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
        if data_send_server:
            data_send_server.append(index)
            data_send_server.append(colour)
            client.send(data_send_server)
            data_send_server = []
        if path_count >= 20:
            if pose[1] >= stop_y:
                print("Reached stopping pose")
                drive.drive(0)
                park(pose, gyro)
                break
        if pi.read(17) == 0:
            break
    print('end')

    