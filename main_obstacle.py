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
from estimate_pose import estimate_pose, reset_pose, get_lidar_pose, lidar_update_pose
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
[[200, 1500], [200, 2000]], # [[200, 1500], [200, 1950]],

# turning point #1
[[200, 2000], [450, 2150.67]], # [[200, 2000], [600, 2200]],
# check colour
[[650, 2300], [650, 2346.7]], # [[700, 2400], [700, 2450]],
# check colour
]

inner_one_section = [
# straight path #1
[[800, 1000], [800, 1200]],
# check colour
[[800, 1200], [800, 1350]], 
# check colour
#turning point #1
[[800, 1400], [800, 2000]],
[[800, 2200], [850, 2200]],
[[850, 2200], [1000, 2200]], # [[850, 2200], [900, 2200]],
# check colour
]

parking_path = [[345, 1350], [345, 2100]]
ccw_parking_path = [[2400, 1600], [2900, 1800]]

unaugmented_obstacle_outer_paths, unaugmented_obstacle_inner_paths = full_path_from_one_section(outer_one_section, inner_one_section)
unaugmented_ccw_obstacle_outer_paths, unaugmented_ccw_obstacle_inner_paths = ccw_paths_from_cw(unaugmented_obstacle_outer_paths, unaugmented_obstacle_inner_paths)

def exit_parking_lot():
    fwd_dist = get_distance(20)
    while fwd_dist < 500:
        # drive.steering(45)
        # drive.drive(150) # 2 seconds
        # drive.steering(-45)
        # drive.drive(-150) # 1-2 seconds
        pass
    print('no parking wall in front')
    # drive.drive(200) # until reached y:1500
    # start following path
    print('exited parking lot')

def park(ldr, pose, gyro):
    # need to add for opposite side
    global parking_path
    print("starting parking procedure")
    reverse_count = 0
    fwd_stop_y = parking_path[1][1]
    rear_stop_y = 1800
    x_min_2 = parking_path[0][0] - 15 #second alignment
    x_max_2 = parking_path[0][0] + 15 #second alignment
    x_min = x_min_2 - 15 #first alignment
    x_max = x_max_2 - 15 #first alignment
    y_min = 1655 - 5
    y_max = 1655 + 5

    print(pose)
    prev_time = time.time()
    parking_start_pos_reached = False
    
    while True:
        if reverse_count >= 3:
                rear_stop_y = parking_path[0][1]
        while pose[1] < fwd_stop_y: #forward
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
            print("parking starting pos pt 1 reached")
            break    

        while pose[1] > rear_stop_y: #backward
            pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)
            navigation.drive_path_back(parking_path, pose, 200)
            reverse_count += 1
            if (time.time() - prev_time) > 0.5:
                print(pose)
                prev_time = time.time()
            if ((x_min < pose[0] < x_max) and (y_min < pose[1] < y_max) and (87 < pose[2] < 93)):
                parking_start_pos_reached = True
                break
        if parking_start_pos_reached:
            drive.drive(0)
            drive.steering(0)
            print("parking starting pos pt 1 reached")
            break 

    stop_time = time.time() + 1
    while time.time() < stop_time:
        ldr.update()
    while ldr.update():
        pass
    pose = get_lidar_pose(ldr)

    print('parking starting pos pt 2')
    parking_start_pos_reached = False
    while True:
        if not ((x_min_2 < pose[0] < x_max_2) and (y_min < pose[1] < y_max) and (87 < pose[2] < 93)):
            while pose[1] < fwd_stop_y:
                pose = lidar_update_pose(pose, gyro, ldr, MM_PER_STEPS)
                navigation.drive_path(parking_path, pose, 200)
                if (time.time() - prev_time) > 0.5:
                    print(pose)
                    prev_time = time.time()
                if ((x_min_2 < pose[0] < x_max_2) and (y_min < pose[1] < y_max) and (87 < pose[2] < 93)):
                    parking_start_pos_reached = True
                    break
            if parking_start_pos_reached:
                drive.drive(0)
                drive.steering(0)
                print("parking starting pos pt 2 reached", pose)
                break
            while pose[1] > rear_stop_y:
                pose = lidar_update_pose(pose, gyro, ldr, MM_PER_STEPS)
                navigation.drive_path_back(parking_path, pose, 200)
                if (time.time() - prev_time) > 0.5:
                    print(pose)
                    prev_time = time.time()
                if ((x_min_2 < pose[0] < x_max_2) and (y_min < pose[1] < y_max) and (87 < pose[2] < 93)):
                    parking_start_pos_reached = True
                    break
            if parking_start_pos_reached:
                drive.drive(0)
                drive.steering(0)
                print("parking starting pos pt 2 reached", pose)
                break 
        else:
            break

    print(get_lidar_pose(ldr))

    pose = [346.53589045596135, 1788.7217848972355, 89.71008969053936]
    print("starting to move into parking zone")
    time.sleep(3)
    # parking 
    # part 1 turn left, move back
    drive.steering(-45)
    drive.drive(-200)

    while pose[1] > 1450:
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

def park_ccw(ldr, pose, gyro):
    global parking_path
    print(parking_path)
    print(pose[1])
    while True:
        while pose[1] < ccw_parking_path[1][1]:
            pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)
            navigation.drive_path(parking_path, pose, 200)
        print("done with path")
        break
    drive.drive(0)
    drive.steering(0)

def run(gyro, ldr, pi):
    global parking_path, obstacle_inner_paths, obstacle_outer_paths, ccw_obstacle_inner_paths, ccw_obstacle_outer_paths
    # client.connect()
    while ldr.update():
        pass
    spike_pose = None
    merged_pose = None
    matches = None
    pose = initial_pose_obstacle(ldr) 
    #stop_y = pose[1] - 50 (orig)
    stop_y = 1510
    print("Initial pose:", pose)
    print("angle_z =", gyro.angle_z())
    L_dist = min(get_distance(ldr, 20), get_distance(ldr, 25), get_distance(ldr, 30), get_distance(ldr, 35), get_distance(ldr, 40), get_distance(ldr, 45))
    R_dist = min(get_distance(ldr, 340), get_distance(ldr, 335), get_distance(ldr, 330), get_distance(ldr, 325), get_distance(ldr, 320), get_distance(ldr, 315))
    path_direction = ""
    if pose[0] < 1500: #left
        obstacle_outer_paths = navigation.augment_paths(unaugmented_obstacle_inner_paths)
        obstacle_inner_paths = navigation.augment_paths(unaugmented_obstacle_outer_paths)
        parking_path = navigation.augment_path(parking_path)
        path_direction = "cw"
        print("clockwise")
    else: #right
        obstacle_inner_paths = navigation.augment_paths(unaugmented_ccw_obstacle_inner_paths)
        obstacle_outer_paths = navigation.augment_paths(unaugmented_ccw_obstacle_outer_paths)
        parking_path = navigation.augment_path(ccw_parking_path)
        path_direction = "ccw"
        print("counter-clockwise")
    paths = obstacle_inner_paths

    index = 0
    print("Paths augmented")
    path_count = 0
    print('steps', drive.steps)
    reset_pose()  
    colour = None

    reverse = False
    print("L_dist: ",  L_dist)
    if pose[0] < 1500: #left
        if R_dist < 400:
            colour = rpicam.detect_blob()
            if colour == "r":
                reverse = True
                print(colour)
                paths = obstacle_outer_paths
            else:
                print(colour)
                paths = obstacle_inner_paths
    else: #right
        if L_dist < 400:
            colour = rpicam.detect_blob()
            if colour == "g":
                reverse = True
                print(colour)
                paths = obstacle_inner_paths
            else:
                print(colour)
                paths = obstacle_outer_paths

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
            if pose[1] <= 1100:
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
                    paths = obstacle_outer_paths
                elif colour == 'g':
                    paths = obstacle_inner_paths
            print('path', paths[count % len(paths)])

        index = count % len(paths)
        if data_send_server:
            data_send_server.append(index)
            data_send_server.append(colour)
            # client.send(data_send_server)
            data_send_server = []
        if path_count >= 20:
            if path_direction == "cw":
                if pose[1] >= stop_y:
                    print("Reached stopping pose")
                    drive.drive(0)
                    park(ldr, pose, gyro)
                    break
            elif path_direction == "ccw":
                print("parking")
                park_ccw(ldr, pose, gyro)
                print("exit function")
                break
        if pi.read(17) == 0:
            break
    print('end')

    