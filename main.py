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
gyro.calibration()
print("Gyro calibrated")

while True:
    print("wait for button")
    while True:
        if pi.read(17) == 0:
            break 

    time.sleep(1)
    if pi.read(17) == 0:
        print("long (open)")
        main_open.run(gyro, ldr, pi)
    else:
        print("short (obstacle)")
        main_obstacle.run(gyro, ldr, pi)



