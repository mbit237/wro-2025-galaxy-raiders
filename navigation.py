import math
import drive

PATH_GAIN = -0.2

def augment_path(path):
    dx = path[1][0] - path[0][0]
    dy = path[1][1] - path[0][1]
    
    dist = math.sqrt(dx**2 + dy**2)
    path_dir = math.atan2(dy, dx) * 180 / math.pi # in degrees
    path_vec = [dx, dy]
    unit_path_vec = [path_vec[0] / dist, path_vec[1] / dist]
    p_unit_path_vec = [-1 * unit_path_vec[1], unit_path_vec[0]] 
    
    path.append(dist)
    path.append(path_dir)
    path.append(unit_path_vec)
    path.append(p_unit_path_vec)
    return path 
    # [start_pos, end_pos, distance, direction, unit_path_vec, perpendicular_unit_path_vec]

def augment_paths(paths):
    for p in range(len(paths)):
        paths[p] = augment_path(paths[p])
    return paths



def dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]

def drive_path(path, pose, speed):
    target_dir = path[3]
    robot_vec = [pose[0] - path[0][0], pose[1] - path[0][1]]
    
    err = dot(path[5], robot_vec) # how far off the robot is (in mm)
    corr = err * PATH_GAIN 
    # Limit correction
    if corr > 30:
        corr = 30
    elif corr < -30:
        corr = -30

    target_dir += corr 
    # print(f"Target direction: {target_dir}, gyro: {pose}")
    drive.steer_p(target_dir, pose[2], speed)

def drive_paths(idx, paths, pose, speed): # idx -- references path currently following 
    path = paths[idx]
    
    drive_path(path, pose, speed)
    robot_vec = [pose[0] - path[0][0], pose[1] - path[0][1]]
    dist_travelled_along_path = dot(path[4], robot_vec)
    
    if dist_travelled_along_path >= path[2]:
        return idx + 1
    else:
        return idx 