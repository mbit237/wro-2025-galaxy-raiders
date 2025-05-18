import pigpio
import time 
import struct
import math 

# import lds02rr
import coind4
from compass import Compass
from gyro import Gyro 
import drive

MM_PER_STEPS = 0.296

ldr = coind4.CoinD4()
# compass = Compass()
# gyro = Gyro() #initialise class
# gyro.calibration()

print_time = time.time() + 2
stop_time = time.time() + 10
ldr.start()
while True:
    ldr.update()

    if time.time() > print_time:
        print(ldr.get_rpm())
        print(ldr.get_measurements())
        print_time = time.time() + 0.5
    # gyro.update_angle()
    # if time.time() > print_time:
    #     print(gyro.angle_z())
    #     print(drive.steps)
    #     print_time = time.time() + 0.5 
    # drive.steer_p(0, gyro.angle_z(), 200)

    # if time.time() > stop_time:
    #     break
    

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