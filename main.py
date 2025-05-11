import pigpio
import time 
import struct
import math 

import lidar
from compass import Compass
from gyro import Gyro 
import drive

MM_PER_STEPS = 0.296

ldr = lidar.LDS02RR(pi)
# compass = Compass()
# gyro = Gyro() #initialise class
# gyro.calibration()

print_time = time.time() + 2
stop_time = time.time() + 10
while True:
    ldr.update()

    if time.time() > print_time:
        print(ldr.get_rpm())
        print_time = time.time() + 0.5
    # gyro.update_angle()
    # if time.time() > print_time:
    #     print(gyro.angle_z())
    #     print(drive.steps)
    #     print_time = time.time() + 0.5 
    # drive.steer_p(0, gyro.angle_z(), 200)

    # if time.time() > stop_time:
    #     break

drive.steer_p(0, 0, 0)
print(drive.steps)
    

## -------- Finding min and max compass angle -------- ##
# if d[0] > lx:
#     lx = d[0]
# elif d[0] < mx:
#     mx = d[0]
# if d[2] > ly:
#     ly = d[2]
# elif d[2] < my:
#     my = d[2]


# Close handle 
# pi.close() -- check 