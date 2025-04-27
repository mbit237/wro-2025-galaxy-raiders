import pigpio
import time 
import struct
import smbus
import math 

MM_PER_STEPS = 0.298 

pi = pigpio.pi()
pi.set_PWM_frequency(20, 200)
def drive(speed): # 0-255
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

def steer_p(dir, curr_angle, speed):
    error = curr_angle - dir
    gain = -1 
    correction = error * gain 
    steering(correction)
    drive(speed)

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

class Compass:
    SCALE_880G = 0b000
    SCALE_1300G = 0b001
    SCALE_1900G = 0b010
    SCALE_2500G = 0b011
    SCALE_4000G = 0b100
    SCALE_4700G = 0b101
    SCALE_5600G = 0b110
    SCALE_8100G = 0b111

    def __init__(self, addr=30, scale=SCALE_880G):
        self.addr = addr 
        self.scale = scale 
        self.init_device()
    
    def init_device(self):
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(self.addr, 0x00, 0b01111000)
        self.bus.write_byte_data(self.addr, 0x01, self.scale << 5) # << shift the bits to the left by 5
        # continuous mode, normal speed I2C
        self.bus.write_byte_data(self.addr, 0x02, 0b00000000)

    def read(self):
        data = self.bus.read_i2c_block_data(self.addr, 0x03, 6) # start from 03 then go to the next 6 bytes 
        return struct.unpack('>hhh', bytes(data)) # big endian, small h means it is positive or negative 
    
    def heading(self):
        x, z, y = self.read()
        sx = (x + 416)*(2/805) - 1 
        sy = (y + 445)*(2/965) - 1
        
        rad = math.atan2(sy, sx)
        
        deg = rad * 180 / math.pi 
        return deg


    

# ------------------------Main-------------------------- #

cb1 = pi.callback(5, pigpio.RISING_EDGE, step_count)
cb2 = pi.callback(6, pigpio.EITHER_EDGE, drive_dir)

gyro = Gyro() #initialise class
gyro.calibration()
# compass = Compass()
print_time = time.time() + 0.5
# LX, LY = 389, 520
# MX, MY = -416, -445
while True:
    gyro.update_angle()
    if time.time() > print_time:
        print(gyro.angle_z())
        print_time = time.time() + 0.5 
    
    steer_p(45, gyro.angle_z(), 120)

    

## -------- Finding min and max compass angle -------- ##
# if d[0] > lx:
#     lx = d[0]
# elif d[0] < mx:
#     mx = d[0]
# if d[2] > ly:
#     ly = d[2]
# elif d[2] < my:
#     my = d[2]