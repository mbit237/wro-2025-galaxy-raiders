import pigpio 
# GPIO 17 

pi = pigpio.pi()
pi.set_mode(17, pigpio.INPUT)
pi.set_pull_up_down(17, pigpio.PUD_UP)
# PUD_DOWN -- opposite of PUD_UP, connected to GND when it is off, connect to 3.3V when button pressed
# PUD_OFF -- no pull up or down (pin is floating, connect it externally to a pull up or down resistor)

while True:
    if pi.read(17) == 0:
        break 

print("DONE")

