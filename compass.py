import math 
import smbus 
import struct 

# Compass 
# LX, LY = 389, 520
# MX, MY = -416, -445

SCALE_880G = 0b000
SCALE_1300G = 0b001
SCALE_1900G = 0b010
SCALE_2500G = 0b011
SCALE_4000G = 0b100
SCALE_4700G = 0b101
SCALE_5600G = 0b110
SCALE_8100G = 0b111

class Compass:
    def __init__(self, addr=30, scale=SCALE_880G):
        self.addr = addr 
        self.scale = scale 
        self.init_device()

    def init_device(self):
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(self.addr, 0x00, 0b01111000) # --> check what is this value
        self.bus.write_byte_data(self.addr, 0x01, self.scale << 5) # << shift the bits to the left by 5
        # continuous mode, normal speed I2C
        self.bus.write_byte_data(self.addr, 0x02, 0b00000000) # --> check what is this value

    def read(self):
        data = self.bus.read_i2c_block_data(self.addr, 0x03, 6) # start from register 03 then go to the next 6 bytes 
        return struct.unpack('>hhh', bytes(data)) # big endian, small h means it is positive or negative 

    def heading(self):
        x, z, y = self.read()
        sx = (x + 416)*(2/805) - 1 
        sy = (y + 445)*(2/965) - 1
        
        rad = math.atan2(sy, sx)
        
        deg = rad * 180 / math.pi 
        return deg