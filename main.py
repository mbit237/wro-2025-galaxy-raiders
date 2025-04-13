import pigpio
import time 
import struct
import smbus

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
    # can use a separate microcontroller (eg. ESP-32) to 
    # read without delays as it does not have a full OS 
    def __init__(self, addr=104):
        self.addr = addr
        self.error_z = -93.99
        self.init_device()
        self.reset_gyro()
        self.reset_prev()

    def reset_prev(self):
        self.prev_z = 0 
        self.prev_time = time.time()

    def reset_gyro(self):
        self.gyro_z = 0 

    def init_device(self):
        # self.handle = pi.i2c_open(1, 104)
        self.bus = smbus.SMBus(1) 
        # pi.i2c_write_byte_data(self.handle, 107, 0)
        self.bus.write_byte_data(self.addr, 107, 0)

    def calibration(self):
        sum_z = 0 
        for x in range(100):
            sum_z += self.rate_z()
            time.sleep(0.1)
        return sum_z/100
        
    def rate_z(self):
        # return struct.unpack('>h', pi.i2c_read_i2c_block_data(self.handle, 71, 2)[1])[0]
        return struct.unpack('>h', bytes(self.bus.read_i2c_block_data(self.addr, 71, 2)))[0]
    
    def angle_z(self):
        return self.gyro_z

    def update_angle(self):
        # z = struct.unpack('>h', pi.i2c_read_i2c_block_data(self.handle, 71, 2)[1])[0]
        z = struct.unpack('>h', bytes(self.bus.read_i2c_block_data(self.addr, 71, 2)))[0]
        z -= self.error_z
        now = time.time()
        delta = now - self.prev_time
        self.gyro_z += (z + self.prev_z)  / 262 * delta
        # self.gyro_z += (z + self.prev_z) / 2  / 131 * delta
        self.prev_z = z
        self.prev_time = now

# ------------------------Main-------------------------- #

cb1 = pi.callback(5, pigpio.RISING_EDGE, step_count)
cb2 = pi.callback(6, pigpio.EITHER_EDGE, drive_dir)

gyro = Gyro() #initialise class
print_time = time.time() + 0.5
while True:
    gyro.update_angle()
    if time.time() > print_time:
        print(gyro.angle_z())
        print_time = time.time() + 0.5 

    

# drive(100)
# time.sleep(20)
# drive(0)
# print(steps)
# drive(-100)
# time.sleep(2)
# drive(0)
# print(steps)