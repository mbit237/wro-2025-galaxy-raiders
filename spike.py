import math


LANDMARKS = [
    [1000, 1000],   #first 4 landmarks are inner landmarks
    [1000, 2000],
    [2000, 1000],
    [2000, 2000],
    [0, 0],        #last 4: outer landmarks
    [0, 3000],
    [3000, 0],
    [3000, 3000]
]

LANDMARK_THRESHOLD = 100
M_THRESHOLD = 100

def identify_spikes(measurements):
    spikes = []
    for d in range(len(measurements)):
        prev_d = measurements[d-1][1]
        curr_d = measurements[d][1]
        next_d = measurements[(d+1) % len(measurements)][1] 
        if prev_d > 100 and curr_d > 100 and next_d > 100:
            if abs((prev_d - curr_d) + (next_d - curr_d)) > M_THRESHOLD:
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
        c_spikes.append([x, y, pose[2], distance])
        
    return c_spikes

def match_landmarks(c_spikes):
    matches = []
    for cs in c_spikes:
        for landmark in LANDMARKS:
            dx = cs[0] - landmark[0]
            dy = cs[1] - landmark[1]
            dist = math.sqrt(dx**2 + dy**2)
            if dist < LANDMARK_THRESHOLD:
                matches.append([cs[0], cs[1], landmark[0], landmark[1], cs[2], cs[3]])
                break
    return matches
