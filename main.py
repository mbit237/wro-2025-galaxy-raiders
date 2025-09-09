#!/bin/python
import pigpio 
import time 
import sys

# chmod a+x 

pi = pigpio.pi()
pi.set_mode(17, pigpio.INPUT) # button -- GPIO17 
pi.set_pull_up_down(17, pigpio.PUD_UP)
# PUD_DOWN -- opposite of PUD_UP, connected to GND when it is off, connect to 3.3V when button pressed
# PUD_OFF -- no pull up or down (pin is floating, connect it externally to a pull up or down resistor)

while True:
    print("wait for button")
    while True:
        if pi.read(17) == 0:
            break 

    time.sleep(1)
    if pi.read(17) == 0:
        print("long (open)")
        sys.exit(0)
    else:
        print("short (obstacle)")
        sys.exit(1)

