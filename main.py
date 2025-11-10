#!/bin/python
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
from initialisation import *
import main_open
import main_obstacle



navigation.PATH_GAIN = -0.5
navigation.MAX_ANGLE = 60
drive.STEER_MAX = 45
drive.CENTER = 50

pi = pigpio.pi()
pi.set_mode(17, pigpio.INPUT)
pi.set_pull_up_down(17, pigpio.PUD_UP)
print('steps', drive.steps)

ldr = coind4.CoinD4() 
ldr.start()
print("Lidar started")

gyro = Gyro() 
gyro.calibration() # keep here for now
print("Gyro calibrated")

while True:
    print("wait for button")
    while True:
        if pi.read(17) == 0:
            break 
    
    # button bounce
    time.sleep(0.1)

    # record time period 
    prev_time = time.time()
    while pi.read(17) == 0:
        pass
    curr_time = time.time()

    time_period = curr_time - prev_time

    # short -- obstacle 
    if time_period < 3:
        print("short (obstacle)")
        main_obstacle.run(gyro, ldr, pi)
        time.sleep(2)
    # long -- open 
    elif time_period < 5:
        print("long (open)")
        main_open.run(gyro, ldr, pi)
        time.sleep(2)
    # long long -- calibrate gyro 
    else:
        print("long long (gyro)")
        time.sleep(1)
        gyro.calibration()
        print("Gyro calibrated")



