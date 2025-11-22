import math
import client_raspi

def full_path_from_one_section(outer_one_section, inner_one_section):
    obstacle_inner_paths = inner_one_section
    obstacle_outer_paths = outer_one_section
    for a in range(5):
        p = inner_one_section[a]
        obstacle_inner_paths.append([[p[0][1], 3000-p[0][0]], [p[1][1], 3000-p[1][0]]])
        p = outer_one_section[a]
        obstacle_outer_paths.append([[p[0][1], 3000-p[0][0]], [p[1][1], 3000-p[1][0]]])
    for a in range(5):
        p = inner_one_section[a]
        obstacle_inner_paths.append([[3000-p[0][0], 3000-p[0][1]], [3000-p[1][0], 3000-p[1][1]]])
        p = outer_one_section[a]
        obstacle_outer_paths.append([[3000-p[0][0], 3000-p[0][1]], [3000-p[1][0], 3000-p[1][1]]])
    for a in range(5):
        p = inner_one_section[a]
        obstacle_inner_paths.append([[3000-p[0][1], p[0][0]], [3000-p[1][1], p[1][0]]])
        p = outer_one_section[a]
        obstacle_outer_paths.append([[3000-p[0][1], p[0][0]], [3000-p[1][1], p[1][0]]])

    print(obstacle_outer_paths, len(obstacle_outer_paths))
    obstacle_outer_paths[0] = [[400, 1000], [400, 1200]]
    obstacle_outer_paths[1] = [[400, 1200], [400, 1350]]
    obstacle_outer_paths[19] = [[600, 700], [559, 700]]
    return obstacle_outer_paths, obstacle_inner_paths
    
def ccw_paths_from_cw(obstacle_outer_paths, obstacle_inner_paths):
    ccw_obstacle_inner_paths = []
    ccw_obstacle_outer_paths = []
    for a in range(len(obstacle_inner_paths)):
        p = obstacle_inner_paths[a]
        ccw_obstacle_inner_paths.append([[3000-p[0][0], p[0][1]], [3000-p[1][0], p[1][1]]])
        p = obstacle_outer_paths[a]
        ccw_obstacle_outer_paths.append([[3000-p[0][0], p[0][1]], [3000-p[1][0], p[1][1]]])

    ccw_obstacle_outer_paths[1] = [[2600, 1200], [2600, 1350]]
    ccw_obstacle_outer_paths[2] = [[2600, 1500], [2600, 2000]]
    return ccw_obstacle_outer_paths, ccw_obstacle_inner_paths

def get_distance(ldr, dir):
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
        if (prev_d - curr_d > 300 or next_d - curr_d > 300) and (curr_d > 250):
            if 250 < measurements[d][1] < 1500:
                closer_spikes.append(measurements[d])  
    
    return closer_spikes

def initial_pose(ldr):
    # forward + backward dist, left + right dist, gyro_z
    # client.send()
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
        if 900 < fwd_dist + rear_dist < 1100:
            break

    y = ((3000 - fwd_dist) + rear_dist) / 2
    print('y', y, fwd_dist, rear_dist)

    #robot is on left side
    vote = 0
    while True:
        if ldr.update():
            identified_spikes = identify_closer_spikes(ldr.get_measurements())
            print('spikes: ', identified_spikes)
            for closer_spike in identified_spikes:
                if 0 < closer_spike[0] < 180:
                    vote += 1
                if 180 < closer_spike[0] < 360:
                    vote -= 1
            print('votes:', vote)
            if vote <= -10: # left side
                x = (left_dist)
                if fwd_dist + rear_dist > 3500:
                    y = 3000 - fwd_dist
                else:
                    y = ((3000 - fwd_dist) + rear_dist) / 2
                print("left side", "x:", x, "y:", y, "fwd_dist:", fwd_dist, "rear_dist:", rear_dist)
                return [x, y, 90]
            if vote >= 10: # right side
                x = ((1000 - right_dist)) + 2000
                if fwd_dist + rear_dist > 3500:
                    y = 3000 - fwd_dist
                else:
                    y = ((3000 - fwd_dist) + rear_dist) / 2
                print("right side", "x:", x, "y:", y, "fwd_dist:", fwd_dist, "rear_dist:", rear_dist)
                return [x, y, 90]

def initial_pose_obstacle(ldr):
    # forward + backward dist, left + right dist, gyro_z
    # client.send()
    fwd_dist = get_distance(ldr, 0)
    left_dist = get_distance(ldr, 90)
    rear_dist = get_distance(ldr, 180)
    right_dist = get_distance(ldr, 270)

    NE_angle = get_distance(ldr, 330)
    SE_angle = get_distance(ldr, 210)
    print('dist at 330deg:', NE_angle, 'dist at 210deg:', SE_angle)
    #robot is on left side
    vote = 0
    while True:
        if ldr.update():
            if (NE_angle > 1750 and SE_angle > 1450) or (NE_angle > 1750 or SE_angle > 1450):
                vote -= 1
            else:
                vote += 1
            print('votes:', vote)
            if vote <= -10: # left side
                x = (left_dist)
                if fwd_dist + rear_dist > 3500:
                    y = 3000 - fwd_dist
                else:
                    y = ((3000 - fwd_dist) + rear_dist) / 2
                print("left side", "x:", x, "y:", y, "fwd_dist:", fwd_dist, "rear_dist:", rear_dist)
                return [x, y, 90]
            if vote >= 10: # right side
                x = ((1000 - right_dist)) + 2000
                if fwd_dist + rear_dist > 3500:
                    y = 3000 - fwd_dist
                else:
                    y = ((3000 - fwd_dist) + rear_dist) / 2
                print("right side", "x:", x, "y:", y, "fwd_dist:", fwd_dist, "rear_dist:", rear_dist)
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

def merge_positions(odo_pose, spike_pose, POSITION_FILTER_RATIO):
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

def merge_heading(merged_position_pose, spike_pose, HEADING_FILTER_RATIO):
    angle_z = merged_position_pose[2]
    spike_z = spike_pose[2]
    merged_angle = angle_z * (1 - HEADING_FILTER_RATIO) + spike_z * HEADING_FILTER_RATIO
    return [merged_position_pose[0], merged_position_pose[1], merged_angle]