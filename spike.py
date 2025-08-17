import math


# LANDMARKS = [
#     [1000, 1000],   #first 4 landmarks are outer landmarks
#     [1000, 2000],
#     [2000, 1000],
#     [2000, 2000],
#     [0, 0],        #last 4: inner landmarks
#     [0, 3000],
#     [3000, 0],
#     [3000, 3000]
# ]
LANDMARKS = [ # open challenge
    [1000, 1000],   #first 4 landmarks are outer landmarks
    [1000, 2000],
    [2000, 1000],
    [2000, 2000],
    [1000, 600],    # additional landmarks for open challenge
    [600, 600],    
    [600, 1000],    
    [600, 2000],
    [600, 2400],
    [1000, 2400],
    [2000, 2400],
    [2400, 2400],
    [2400, 2000],
    [2400, 1000],
    [2400, 600],
    [2000, 600],
    [0, 0],        #next 4: inner landmarks
    [0, 3000],
    [3000, 0],
    [3000, 3000],
]

LANDMARKS = [ # obstacle challenge
    [1000, 1000],   #first 4 landmarks are outer landmarks
    [1000, 2000],
    [2000, 1000],
    [2000, 2000],
    [0, 0],        #last 4: inner landmarks
    [0, 3000],
    [3000, 0],
    [3000, 3000]
]

LANDMARK_THRESHOLD = 120
M_THRESHOLD = 100

def identify_spikes(measurements):
    spikes = []
    for d in range(len(measurements)):
        prev_d = measurements[d-1][1]
        curr_d = measurements[d][1]
        next_d = measurements[(d+1) % len(measurements)][1] 
        if prev_d < 250 or next_d < 250 or curr_d < 250:  # if previous or next distance is too small, skip this measurement
            continue
        if abs((prev_d - curr_d) + (next_d - curr_d)) > M_THRESHOLD:
            if 100 < measurements[d][1] < 1500:
                spikes.append(measurements[d])  
            
    return spikes

def add_cartesian(pose, spikes):# spikes in cartesian coordinates
    c_spikes = []
    for spike in spikes:
        distance = spike[1]
        theta = (pose[2] + spike[0]) * math.pi / 180 # in radians, robot angle + lidar's spike angle
        dx = distance * math.cos(theta)
        dy = distance * math.sin(theta)
    
        x = pose[0] + dx
        y = pose[1] + dy
        c_spikes.append([x, y, spike[0], distance])
        
    return c_spikes

def match_landmarks(c_spikes):
    matches = []
    for cs in c_spikes:
        for landmark in LANDMARKS:
            dx = cs[0] - landmark[0]
            dy = cs[1] - landmark[1]
            dist = math.sqrt(dx**2 + dy**2)
            err_x = round(abs(cs[0] - landmark[0]), 2)
            err_y = round(abs(cs[1] - landmark[1]), 2)
            if dist < LANDMARK_THRESHOLD:
                # matches.append([round(cs[0], 2), round(cs[1], 2), landmark[0], landmark[1], cs[2], cs[3]])
                matches.append([round(cs[0], 2), round(cs[1], 2), landmark[0], landmark[1], cs[2], cs[3], err_x, err_y])
                break
    return matches
