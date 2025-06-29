import math
import time
import drive

prev_steps_count = 0
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
    curr_heading = (delta_z + prev_z)  / 262 * delta + pose[2]  # current heading in degrees
    aver_heading = (curr_heading + pose[2]) / 2
    prev_z = delta_z  # update previous z for next iteration
    
    # change in x and y, dist_travelled is the hypotenuse
    dx = dist_travelled * math.cos(aver_heading / 180 * math.pi) # returns the opposite
    dy = dist_travelled * math.sin(aver_heading / 180 * math.pi) # returns the adjacent

    return [pose[0] + dx, pose[1] + dy, curr_heading]