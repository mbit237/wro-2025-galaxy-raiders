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
import rpicam

MM_PER_STEPS = 0.296

# ratios need to be tuned
POSITION_FILTER_RATIO = 0.1 # 0.1% confidence
HEADING_FILTER_RATIO = 0.01  # 0.1%, if it is too low, heading error will be larger
                             # if too high, robot will jump around

navigation.PATH_GAIN = -0.5
navigation.MAX_ANGLE = 60
drive.STEER_MAX = 45
drive.CENTER = 50

# paths = [
#     [[500, 500], [500, 2000]], 
#     [[500, 2500], [2000, 2500]], 
#     [[2500, 2500], [2500, 1000]], 
#     [[2500, 500], [1000, 500]]
# ]


cw_paths = [ # clockwise paths for open challenge
    # [[500, 500], [500, 2000]], 
    # [[500, 2500], [2000, 2500]], 
    # [[2500, 2500], [2500, 1000]], 
    # [[2500, 500], [1000, 500]],
    [[300, 300], [300, 2350]],
    [[300, 2700], [2350, 2700]],
    [[2700, 2700], [2700, 650]],
    [[2700, 300], [650, 300]]
]

ccw_paths = [ # counter-clockwise paths for open challenge
    # [[500, 500], [500, 2000]], 
    # [[500, 2500], [2000, 2500]], 
    # [[2500, 2500], [2500, 1000]], 
    # [[2500, 500], [1000, 500]],
    [[2700, 300], [2700, 2350]],
    [[2700, 2700], [650, 2700]],
    [[300, 2700], [300, 650]],
    [[300, 300], [2350, 300]]
]

obstacle_outer_paths = [
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

# obstacle_inner_paths = [
# # straight path #1
# [[800, 1000], [800, 1200]],
# # check colour
# [[800, 1500], [800, 2000]],
# # check colour

# #turning point #1
# [[800, 2200], [850, 2200]],
# [[850, 2200], [900, 2200]],
# # check colour
# ]
obstacle_inner_paths = [
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
ccw_obstacle_outer_paths = []
ccw_obstacle_inner_paths = []

for a in range(5):
    p = obstacle_inner_paths[a]
    obstacle_inner_paths.append([[p[0][1], 3000-p[0][0]], [p[1][1], 3000-p[1][0]]])
    p = obstacle_outer_paths[a]
    obstacle_outer_paths.append([[p[0][1], 3000-p[0][0]], [p[1][1], 3000-p[1][0]]])
for a in range(5):
    p = obstacle_inner_paths[a]
    obstacle_inner_paths.append([[3000-p[0][0], 3000-p[0][1]], [3000-p[1][0], 3000-p[1][1]]])
    p = obstacle_outer_paths[a]
    obstacle_outer_paths.append([[3000-p[0][0], 3000-p[0][1]], [3000-p[1][0], 3000-p[1][1]]])
for a in range(5):
    p = obstacle_inner_paths[a]
    obstacle_inner_paths.append([[3000-p[0][1], p[0][0]], [3000-p[1][1], p[1][0]]])
    p = obstacle_outer_paths[a]
    obstacle_outer_paths.append([[3000-p[0][1], p[0][0]], [3000-p[1][1], p[1][0]]])

for a in range(len(obstacle_inner_paths)):
    p = obstacle_inner_paths[a]
    ccw_obstacle_inner_paths.append([[3000-p[0][0], p[0][1]], [3000-p[1][0], p[1][1]]])
    p = obstacle_outer_paths[a]
    ccw_obstacle_outer_paths.append([[3000-p[0][0], p[0][1]], [3000-p[1][0], p[1][1]]])

obstacle_outer_paths[0] = [[400, 1000], [400, 1200]]
obstacle_outer_paths[1] = [[400, 1200], [400, 1350]]
obstacle_outer_paths[19] = [[600, 700], [559, 700]]
ccw_obstacle_outer_paths[1] = [[2600, 1200], [2600, 1350]]
ccw_obstacle_outer_paths[2] = [[2600, 1500], [2600, 2000]]

# for b in range(4):
#     p = ccw_obstacle_inner_paths[]


# obstacle_outer_paths = [
# # straight path #1
# [[200, 1000], [200, 1350]],
# # check colour
# [[200, 1500], [200, 2000]],

# # turning point #1
# [[200, 2000], [600, 2200]],
# # check colour
# [[700, 2400], [700, 2450]],
# # check colour

# # top path #2
# [[800, 2800], [1350, 2800]],
# # check colour
# [[1500, 2800], [2000, 2800]],

# # turning point #2
# [[2000, 2800], [2400, 2200]],
# # check colour
# [[2400, 2200], [2800, 2000]],
# # check colour

# # right path #3
# [[2800, 2000], [2800, 1650]],
# # check colour
# [[2800, 1500], [2800, 1000]],

# # turning point #3
# [[2800, 1000], [2400, 600]],
# # check colour
# [[2400, 600], [2000, 200]],
# # check colour

# # bottom path #4
# [[2000, 200], [1650, 200]],
# # check colour
# [[1500, 200], [1000, 200]],

# # turning point #4
# [[1000, 200], [600, 600]],
# # check colour
# [[600, 600], [200, 1000]],
# # check colour
# ]

# obstacle_inner_paths = [
# # straight path #1
# [[800, 1000], [800, 1350]],
# # check colour
# [[800, 1500], [800, 2000]],
# # check colour

# #turning point #1
# [[800, 2200], [900, 2200]], # have same no. of inner and outer paths, easier paths swtitcing
# # check colour
# [[1000, 2200], [1350, 2200]],

# #straight path #2
# [[1350, 2200], [2000, 2200]],
# # check colour
# [[1500, 2200], [1850, 2200]],
# # check colour

# # turning point #2
# [[1850, 2200], [2000, 2200]],
# # check colour
# [[2000, 2200], [2200, 2000]],

# # straight path #3
# [[2200, 2000], [2200, 1650]],
# # check colour
# [[2200, 1500], [2200, 1150]],
# # check colour

# # turning point #3
# [[2200, 1150], [2200, 1000]],
# # check colour
# [[2200, 1000], [2000, 800]],

# # straight path #4
# [[2000, 800], [1650, 800]],
# # check colour
# [[1500, 800], [1150, 800]],
# # check colour

# # turning point #4
# [[1150, 800], [1000, 800]],
# # check colour
# [[1000, 800], [800, 1000]], # go back to start
# ]

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
                print("left side", "x:", x, "y:", y)
                return [x, y, 90]
            if vote >= 10: # right side
                x = ((1000 - right_dist)) + 2000
                y = ((3000 - fwd_dist) + rear_dist) / 2
                print("right side", "x:", x, "y:", y)
                return [x, y, 90]

def calc_position_error(matches):
    if not matches:  # Handle empty matches list
        return [0, 0]
    errors = []
    for match in matches:
        errorx = match[0] - match[2]
        errory = match[1] - match[3]
        errors.append([errorx, errory])
        
    err_x = 0
    err_y = 0
    for err in errors:
        err_x += err[0]
        err_y += err[1]
    ave_err_x = err_x / len(errors)
    ave_err_y = err_y / len(errors)
    
    return [ave_err_x, ave_err_y]

def calc_pose(pose, error):
    new_x = pose[0] - error[0]
    new_y = pose[1] - error[1]
    
    return [new_x, new_y, pose[2]]

def merge_positions(odo_pose, spike_pose):
    odo_x = odo_pose[0]
    odo_y = odo_pose[1]
    spike_x = spike_pose[0]
    spike_y = spike_pose[1]
    merged_x = odo_x * (1 - POSITION_FILTER_RATIO) + spike_x * POSITION_FILTER_RATIO
    merged_y = odo_y * (1 - POSITION_FILTER_RATIO) + spike_y * POSITION_FILTER_RATIO
    
    return [merged_x, merged_y, odo_pose[2]]

def calc_angle_error(pose, matches):
    if not matches:
        return 0
    
    angle_errors = []
    
    for match in matches:
        robot_x = pose[0]
        robot_y = pose[1]
        landmark_x = match[2]
        landmark_y = match[3]
        dx = landmark_x - robot_x
        dy = landmark_y - robot_y
        a_theta = math.atan2(dy, dx) * 180 / math.pi # actual theta
        m_theta = pose[2] + match[4] # measured_theta
        angle_err = m_theta - a_theta
        while angle_err > 180:
            angle_err -= 360
        while angle_err < -180:
            angle_err += 360
        
        angle_errors.append(angle_err)
    # print(angle_errors)
    ave_angle_err = sum(angle_errors) / len(angle_errors)
    return ave_angle_err

def calc_heading(pose, error):
    return [pose[0], pose[1], (pose[2] - error)]

def merge_heading(merged_position_pose, spike_pose):
    angle_z = merged_position_pose[2]
    spike_z = spike_pose[2]
    merged_angle = angle_z * (1 - HEADING_FILTER_RATIO) + spike_z * HEADING_FILTER_RATIO
    return [merged_position_pose[0], merged_position_pose[1], merged_angle]

pi = pigpio.pi()
pi.set_mode(17, pigpio.INPUT)
pi.set_pull_up_down(17, pigpio.PUD_UP)
print('steps', drive.steps)
# while True:
#     if ldr.update():
#         break
# print('cleared')
ldr = coind4.CoinD4() #lidar initialise
ldr.start()
print("Lidar started")
L_dist = min(get_distance(20), get_distance(25), get_distance(30), get_distance(35), get_distance(40), get_distance(45))
R_dist = min(get_distance(340), get_distance(335), get_distance(330), get_distance(325), get_distance(320), get_distance(315))

# compass = Compass()
gyro = Gyro() #initialise class
gyro.calibration()
print("Gyro calibrated")

# print('start')

# client.connect()
# print("Client connected")

# while True:
#     initial_pose()
#     print('done')
    # pose = estimate_pose(initial_pose(), gyro.delta_z(), MM_PER_STEPS)
    # if ldr.update():
    #     print("working")
    #     lidar_measurements = ldr.get_measurements()
    #     closer_spikes = identify_closer_spikes(lidar_measurements)
    #     print(closer_spikes)

pose = initial_pose() # Initial pose (mm)
stop_y = pose[1] - 50
print("Initial pose:", pose)
print("angle_z =", gyro.angle_z())

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

# pose = [500, 500, 90] # Initial pose for testing
# time.sleep(2)
# count = 0
path_count = 0
print('steps', drive.steps)
reset_pose()  # Reset the pose to the initial state
colour = None

reverse = False
if pose[0] < 1500: #left
    if R_dist < 400:
        colour = rpicam.detect_blob()
        if colour == "r":
            reverse = True
            paths = obstacle_inner_paths
        else:
            paths = obstacle_outer_paths
else:
    if L_dist < 400:
        colour = rpicam.detect_blob()
        if colour == "g":
            reverse = True
            paths = obstacle_outer_paths
        else:
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
        merged_position_pose = merge_positions(pose, spike_pose)
        angle_error = calc_angle_error(merged_position_pose, matches)
        # print(angle_error)
        spike_heading_pose = calc_heading(merged_position_pose, angle_error)
        # print(spike_heading_pose)
        merged_pose = merge_heading(merged_position_pose, spike_heading_pose)
        # print("merged_position_pose:", merged_position_pose[2], "spike_heading_pose:", spike_heading_pose[2])
        # pose = merged_position_pose
        pose = merged_pose

    
 
    if pose[0] < 1500: #left
        drive.steer_p_back(90, pose[2], 200)
        if pose[1] <= 1000:
            break
    else: #right
        drive.steer_p_back(90, pose[2], 200)
        if pose[1] <= 1500:
            break 

while True:
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS) 
    # print("Pose:", pose)
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
        merged_position_pose = merge_positions(pose, spike_pose)
        angle_error = calc_angle_error(merged_position_pose, matches)
        # print(angle_error)
        spike_heading_pose = calc_heading(merged_position_pose, angle_error)
        # print(spike_heading_pose)
        merged_pose = merge_heading(merged_position_pose, spike_heading_pose)
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
    if path_count >= 40:
        if pose[1] >= stop_y:
            print("Reached stopping pose")
            break
    if pi.read(17) == 0:
        break

drive.drive(0) 
drive.steering(0)
'''
if red:
    if path count
else:
    switch to outer path
'''