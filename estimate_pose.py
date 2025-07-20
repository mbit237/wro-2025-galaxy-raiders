import math
import time
import drive

prev_steps_count = 0
prev_z = 0
prev_time = time.time() 
# c = dist_travelled by wheel
# theta = theta2 - theta1
# r = c / theta
# dy = r * (math.sin(theta))
# dx = r - (r * math.cos(theta))

def estimate_pose(pose, delta_z, MM_PER_STEPS=0.296):
    global prev_steps_count, prev_z, prev_time
    curr_steps_count = drive.steps
    dist_travelled = (curr_steps_count - prev_steps_count) * MM_PER_STEPS # distance travelled since last estimate (in mm)
    prev_steps_count = curr_steps_count

    now = time.time()
    delta = now - prev_time
    prev_time = now
    
    #adding the change in gyro heading to previous pose heading
    theta = (delta_z + prev_z) / 262 * delta
    curr_heading = theta + pose[2]  # current heading in degrees
    prev_z = delta_z  # update previous z for next iteration
    
    if theta < 0.001 and theta < -0.001:  # if theta is too small, don't change pose
        dx = 0
        dy = dist_travelled
    else:
        theta = math.radians(theta)  # convert to radians
        r = dist_travelled / theta
        dy = r * math.sin(theta)  # change in y
        dx = r - r * math.cos(theta) # change in x

    v1 = [math.cos(math.radians(pose[2])), math.sin(math.radians(pose[2]))]
    v2 = [v1[1], -v1[0]]  # rotate 90 degrees for dx
    x = pose[0] + v1[0] * dy + v2[0] * dx
    y = pose[1] + v1[1] * dy + v2[1] * dx

    return [x, y, curr_heading]

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