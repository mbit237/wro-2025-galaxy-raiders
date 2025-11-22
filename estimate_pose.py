import math
import time
import drive
from initialisation import get_distance

LIDAR_WHEEL_DIST = 195

prev_steps_count = 0
prev_z = 0
prev_time = time.time() 
# c = dist_travelled by wheel
# theta = theta2 - theta1
# r = c / theta
# dy = r * (math.sin(theta))
# dx = r - (r * math.cos(theta))

def reset_pose():
    global prev_steps_count, prev_z, prev_time
    prev_steps_count = drive.steps
    prev_z = 0
    prev_time = time.time()

def estimate_pose(pose, delta_z, MM_PER_STEPS=0.296):
    global prev_steps_count, prev_z, prev_time
    curr_steps_count = drive.steps
    dist_travelled = (curr_steps_count - prev_steps_count) * MM_PER_STEPS # distance travelled since last estimate (in mm)
    prev_steps_count = curr_steps_count

    now = time.time()
    delta = now - prev_time
    prev_time = now
    
    #adding the change in gyro heading to previous pose heading
    theta = (delta_z + prev_z) / 2 / 131 * delta # tune rate of rotation (131)
    curr_heading = theta + pose[2]  # current heading in degrees
    prev_z = delta_z  # update previous z for next iteration

    # calculate heading and perpendicular vectors for initial position
    v1 = [math.cos(math.radians(pose[2])), math.sin(math.radians(pose[2]))]
    v2 = [v1[1], -v1[0]]  # rotate 90 degrees for dx

    # calculate initial pose of rear wheels
    wheel_pose = [pose[0] - LIDAR_WHEEL_DIST * v1[0], pose[1] - LIDAR_WHEEL_DIST * v1[1], pose[2]]
    # print("Wheel pose:", wheel_pose)

    if theta < 0.001 and theta < -0.001:  # if theta is too small, don't change pose
        dx = 0
        dy = dist_travelled
    else:
        theta = math.radians(theta)  # convert to radians
        r = dist_travelled / theta
        dy = r * math.sin(theta)  # change in y
        dx = r * math.cos(theta) - r # change in x, final - initial 

    x = wheel_pose[0] + v1[0] * dy + v2[0] * dx 
    y = wheel_pose[1] + v1[1] * dy + v2[1] * dx

    # print("X:", x, "Y:", y)

    # calculate final heading vector
    curr_heading_rad = math.radians(curr_heading)
    v3 = [math.cos(curr_heading_rad), math.sin(curr_heading_rad)]

    # shifted pose back to lidar pose 
    x_final = x + LIDAR_WHEEL_DIST * v3[0] 
    y_final = y + LIDAR_WHEEL_DIST * v3[1]
    # print("Final pose:", [x_final, y_final, curr_heading])

    return [x_final, y_final, curr_heading]

def estimate_pose_old(pose, delta_z, MM_PER_STEPS=0.296):
    global prev_steps_count, prev_z, prev_time
    curr_steps_count = drive.steps
    dist_travelled = (curr_steps_count - prev_steps_count) * MM_PER_STEPS # distance travelled since last estimate (in mm)
    prev_steps_count = curr_steps_count

    now = time.time()
    delta = now - prev_time
    prev_time = now
    
    #adding the change in gyro heading to previous pose heading
    curr_heading = (delta_z + prev_z)  / 262 * delta + pose[2]  # current heading in degrees
    aver_heading = (curr_heading + pose[2]) / 2
    prev_z = delta_z  # update previous z for next iteration
    
    # change in x and y, dist_travelled is the hypotenuse
    dx = dist_travelled * math.cos(aver_heading / 180 * math.pi) # returns the opposite
    dy = dist_travelled * math.sin(aver_heading / 180 * math.pi) # returns the adjacent

    return [pose[0] + dx, pose[1] + dy, curr_heading]

def get_lidar_pose(ldr):
    while True:
        fwd_dist = get_distance(ldr, 0)
        rear_dist = get_distance(ldr, 180)
        if 2900 < fwd_dist + rear_dist < 3100:
            break

    y = ((3000 - fwd_dist) + rear_dist) / 2
    print('y', y, fwd_dist, rear_dist)
    
    while True:
        left_dist = get_distance(ldr, 90)
        right_dist = get_distance(ldr, 270)
        if 900 < left_dist + right_dist < 1100:
            break

    x = (left_dist + (1000 - right_dist)) / 2
    print('x', x, left_dist, right_dist)
    return [x, y, 90]

def lidar_update_pose(pose, gyro, ldr, MM_PER_STEPS):
    pose = estimate_pose(pose, gyro.delta_z(), MM_PER_STEPS)
    if ldr.update():
        lidar_measurements = ldr.get_measurements()
        fwd_dist = 0
        rear_dist = 0
        left_dist = 0
        right_dist = 0
        for m in lidar_measurements:
            if 1 > m[0] > -1:
                fwd_dist = m[1]
        for m in lidar_measurements:
            if 181 > m[0] > 179:
                rear_dist = m[1]

        if 2900 < rear_dist + fwd_dist < 3100:
            pose[1] = ((3000 - fwd_dist) + rear_dist) / 2
            print('pose[1] updated with lidar')
        
        for m in lidar_measurements:
            if 271 > m[0] > 269:
                right_dist = m[1]
        for m in lidar_measurements:
            if 91 > m[0] > 89:
                left_dist = m[1]

        if 900 < left_dist + right_dist < 1100:
            pose[0] = ((1000 - right_dist) + left_dist) / 2
            print('pose[1] updated with lidar')
        print(pose, left_dist, right_dist)

    return pose                
