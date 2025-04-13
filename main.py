import pigpio
import time 
import struct

MM_PER_STEPS = 0.298 

pi = pigpio.pi()
pi.set_PWM_frequency(20, 200)
def drive(speed):
    if speed > 0:
        pi.set_PWM_dutycycle(20, 255-speed)
        pi.set_PWM_dutycycle(21, 255)
    else:
        pi.set_PWM_dutycycle(20, 255)
        pi.set_PWM_dutycycle(21, 255+speed)

def steering(dir):
    if dir < -45:
        dir = -45
    elif dir > 45:
        dir = 45
    pulse_duration = 1400 + (80 / 9) *(dir + 45) - 400
    pi.set_servo_pulsewidth(14, pulse_duration)

steps = 0
pin6_level = False

def step_count(gpio, level, tick):
    global steps
    # if pin6_level:
    #     steps -= 1
    # else:
        # steps += 1
    steps += 1

def drive_dir(gpio, level, tick):
    global pin6_level 
    if level == 1:
        pin6_level = True 
    elif level == 0:
        pin6_level = False 

class Gyro:
    def __init__(self, addr=104):
        self.init_device()
        self.addr = addr

    def init_device(self):
        self.handle = pi.i2c_open(1, 104)
        self.pi.i2c_write_byte_data(self.handle, 107, 0)

    def rate_z(self):
        return struct.unpack('>h', pi.i2c_read_i2c_block_data(self.handle, 71, 2)[1])

cb1 = pi.callback(5, pigpio.RISING_EDGE, step_count)
cb2 = pi.callback(6, pigpio.EITHER_EDGE, drive_dir)

gyro = Gyro() #initialise class

# drive(100)
# time.sleep(20)
# drive(0)
# print(steps)
# drive(-100)
# time.sleep(2)
# drive(0)
# print(steps)