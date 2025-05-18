import serial 
import struct

CMD_HEADER = b'\xAA\x55'
DATA_HEADER = b'\xAA\x55'
HEADER_LENGTH = 10
FRAME_LENGTH = 85

class CoinD4: # standard convention to start classes with uppercase 
    def __init__(self, uart, strength=False, integer=False): 
        # strength do not need -> intensity of signal
        #  - look at it to determine whether the reading is accurate 
        self.uart = serial.Serial('/dev/ttyS0', 230400, timeout=0)
        self.strength = strength
        self.integer = integer
        self.buf = bytearray(FRAME_LENGTH)
        self.ptr = 0
        self.speed = 0
        self.measurements = []
        self.sample_count = 0
        self.measurement_ptr = 0
        self._prev_start_angle = 0
        # if self.integer: -- needed if using microcontroller that is unable ot handle float values well 
        #     for _ in range(360): # initalise 
        #         self.measurements.append(-1)
        # else:
        #     for _ in range(500):
        #         if strength:
        #             self.measurements.append([0.0, -1, 0])
        #         else:
        #             self.measurements.append([0.0, -1])


    def update(self):
        while self.uart.in_waiting > 0:
            chars = self.uart.read(100)

            for char in chars:
                if self.ptr == 0:
                    if char == DATA_HEADER[0]: # check if it matches first character
                        self.buf[0] = char
                        self.ptr += 1
                elif self.ptr == 1:
                    if char == DATA_HEADER[1]: # check match second character
                        self.buf[1] = char
                        self.ptr += 1
                    else:
                        self.ptr = 0
                else:
                    self.buf[self.ptr] = char
                    self.ptr += 1
                    if self.ptr == HEADER_LENGTH:
                        self.sample_count = self.buf[3] # 4th byte is sample count, it is variable 
                        if self.sample_count > 25 or self.sample_count == 0: # error 
                            self.ptr = 0
                    elif self.ptr == HEADER_LENGTH + self.sample_count * 3:
                        self.ptr = 0
                        if self._checksum_correct():
                            return self._parse_frame()
        return False

    def _send_cmd(self, code):
        cmd = bytearray(4)
        cmd[0:2] = CMD_HEADER
        cmd[2] = code
        cmd[3] = cmd[0] ^ cmd[1] ^ cmd[2]
        self.uart.write(cmd)

    def start(self):
        self._send_cmd(0xF0)

    def set_high_exposure(self):
        self._send_cmd(0xF1)

    def set_low_exposure(self):
        self._send_cmd(0xF2)

    def stop(self):
        self._send_cmd(0xF5)

    def get_measurements(self):
        return self.measurements

    def get_rpm(self):
        return self.speed * 60

    def _parse_frame(self):
        start_angle, end_angle = struct.unpack('<HH', self.buf[4:8])
        start_angle = (start_angle >> 1) / 64
        end_angle = (end_angle >> 1) / 64
        if self.buf[2] & 1:
            self.speed = (self.buf[2] >> 1) / 10

        if start_angle < self._prev_start_angle and self.integer == False:
            self.measurements = []
        self._prev_start_angle = start_angle

        if self.sample_count > 1:
            angle_step = (end_angle - start_angle) / (self.sample_count - 1)
        else:
            angle_step = 0

        for i in range(self.sample_count): # go through all the samples
            start_index = HEADER_LENGTH + i * 3 # each sample has 3 bytes  
            exposure_mode = 0x03 & self.buf[start_index] # unused  
            distance = self.buf[start_index+1] >> 2 | self.buf[start_index+2] << 6 
            # << 6 is the same as mulitiply by 64 
            # each time we shift is mulitplying by 2 
            angle = start_angle + i * angle_step
            if self.integer:
                self.measurements[round(angle) % 360] = distance
            else:
                self.measurements.append([angle, distance]) # distance is in millimeters 
                # if self.strength:
                #     self.measurements[self.measurement_ptr][2] = self.buf[start_index+2] >> 2 | (0x03 & self.buf[start_index+1]) << 6

        if end_angle > 345: # identify last reading 
            # 345 -- experimentation 
            return True
        return False

    def _checksum_correct(self): # checksum algorithm 
        cs = self.buf[1] ^ self.buf[3] ^ self.buf[5] ^ self.buf[7] # XOR: heading and the samples (every two steps)
        for i in range(self.sample_count):
            cs ^= self.buf[10+i*3+2]
        if cs != self.buf[9]:
            return False

        cs = self.buf[0] ^ self.buf[2] ^ self.buf[4] ^ self.buf[6]
        for i in range(self.sample_count):
            cs ^= self.buf[10+i*3]
            cs ^= self.buf[10+i*3+1]
        if cs != self.buf[8]:
            return False

        return True