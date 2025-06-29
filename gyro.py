import time 
import smbus
import struct

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
        for x in range(500):
            sum_z += self.rate_z()
            time.sleep(0.01)
        self.error_z = sum_z / 1000
        return sum_z/1000

    def save_calibration(self, filename='gyro_calibration.txt'):
        with open(filename, 'w') as f:
            f.write(str(self.error_z))

    def load_calibration(self, filename='gyro_calibration.txt'):
        try:
            with open(filename, 'r') as f:
                self.error_z = float(f.read())
        except FileNotFoundError:
            pass

    def rate_z(self):
        # return struct.unpack('>h', pi.i2c_read_i2c_block_data(self.handle, 71, 2)[1])[0]
        return struct.unpack('>h', bytes(self.bus.read_i2c_block_data(self.addr, 71, 2)))[0]
    
    def angle_z(self):
        return self.gyro_z

    def delta_z(self):
        # z = struct.unpack('>h', pi.i2c_read_i2c_block_data(self.handle, 71, 2)[1])[0]
        z = struct.unpack('>h', bytes(self.bus.read_i2c_block_data(self.addr, 71, 2)))[0]
        z -= self.error_z
        return z

    def update_angle(self):
        # z = struct.unpack('>h', pi.i2c_read_i2c_block_data(self.handle, 71, 2)[1])[0]
        z = struct.unpack('>h', bytes(self.bus.read_i2c_block_data(self.addr, 71, 2)))[0]
        z -= self.error_z
        now = time.time()
        delta = now - self.prev_time
        self.gyro_z += (z + self.prev_z)  / 262 * delta 
        # self.gyro_z += (z + self.prev_z) / 2  / 131 * delta
        # z -- rate of rotation 
        # average of the previous z and current z * time = number of degrees turned 
        # the z is measured to another unit: divide it by 131 to convert it to degrees 
        self.prev_z = z
        self.prev_time = now