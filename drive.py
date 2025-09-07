import pigpio

STEER_MAX = 45
CENTER = 45

#GPIO20, GPIO21

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
    if dir < -STEER_MAX:
        dir = -STEER_MAX
    elif dir > STEER_MAX:
        dir = STEER_MAX
    pulse_duration = 1340 + (80 / 9) *(dir + CENTER) - 400
    pi.set_servo_pulsewidth(23, pulse_duration)

steps = 0
pin6_level = False
# gpio -- pin 
# level -- rising / falling edge 
# tick -- The number of microseconds since boot
def step_count(gpio, level, tick):
    global steps
    if pin6_level:
        steps += 1
    else:
        steps -= 1
    # steps += 1

def drive_dir(gpio, level, tick):
    global pin6_level 
    if level == 1:
        pin6_level = True 
    elif level == 0:
        pin6_level = False 

def steer_p(dir, curr_angle, speed):
    while curr_angle - dir > 180:
        curr_angle -= 360
    while curr_angle - dir < -180:
        curr_angle += 360

    error = curr_angle - dir
    gain = 2 
    correction = error * gain 
    steering(correction)
    drive(speed)

def steer_p_back(dir, curr_angle, speed):
    while curr_angle - dir > 180:
        curr_angle -= 360
    while curr_angle - dir < -180:
        curr_angle += 360

    error = curr_angle - dir
    gain = -2 
    correction = error * gain 
    steering(correction)
    drive(-speed)

cb1 = pi.callback(5, pigpio.RISING_EDGE, step_count)
cb2 = pi.callback(6, pigpio.EITHER_EDGE, drive_dir)