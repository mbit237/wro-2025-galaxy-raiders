# wro-2025-galaxy-raiders
# Solution Summary
# This repository contains the full stack for the robot: electronics (PCB design and fabrication artifacts), sensors (2D LiDAR, gyro, wheel encoders, compass), decision logic (landmark-based localisation and odometry that are fused using a complementary filter), navigation (path planning, proportional steering), colour detection for the Obstacle Challenge (Raspberry Pi Camera–based colour blob detection for green and red), and a set of utilities for calibration, plotting, as well as testing.

# It is implemented in Python and designed to run on a Raspberry Pi 4b which is the primary controller. The Pi interfaces directly with the sensors (LiDAR via serial/USB, MPU6050 gyro via I²C, wheel encoders via GPIO), and controls the navigation (continuous DC drive through a motor driver; steering via a PWM-controlled servo). The custom PCB consolidates power distribution, connectors, and signal routing. 

# During the run, the program reads initial ranges to estimate the robot’s start pose, continuously merges odometry with landmark corrections, follows the active path, and stops automatically once a path-segment counter reaches the configured limit.

# PCB (wro-pcb-v2 folder)
# Contains everything needed to fabricate and assemble the controller board:
# Schematic & PCB: Source files for the custom board (signal routing for power in, motor driver, servo PWM, I²C, UART/USB headers for sensors, encoder inputs).
# Drill / CAM: Drill files and drill G-code for CNC milling; SVGs of the board outline and copper to be milled with LaserWeb 
# Connected components
# Power: Battery input (screw terminal), voltage regulator for motor driver, and servo.
# Actuators: Screw terminals/headers to the motor driver output and a 3-pin servo header (GND, +5/6V BEC, PWM).
# Sensors: I²C header for MPU6050; UART header for LiDAR; encoder inputs to GPIO connectors.
# Tactile button input to GPIO for initialisation 

# Localisation using sensors:
# coind4: functions for current lidar, parsing in each data frame and checking the checksum 
# gyro.py (mpu6050): basic functions for gyro to update heading
# gyro_calibration_saver.py:
# [unused] lds02rr.py: functions for previous lidar, also pareses in each data frame and checks the # checksum
# [unused] compass.py: functions for compass module to read its heading, not implemented in current version

# Navigation 
# spike.py: functions to match to lidar points to landmarks 
# estimate_pose.py: functions for odometry, which uses wheel rotation and gyro to estimate the robot's position and heading while it is running. New estimate_pose function uses vectors
# drive.py: navigation functions 
# step_count: receive signals from encoder to calculate number of steps after a certain period of time 
# steering_p: driving servo motor with pwm signal that is derived from the direction in degrees
# navigation.py: functions to augment robot’s paths and drive to the path which utilises proportional control to avoid steering off path

# Obstacle Challenge
# Using camera to identify the position of the red and green blocks to know which path to take at each decision point, ie after the turn at each corner
# rpicam.py: functions for detection of red and green blobs based on HSV thresholds using opencv
# rpicam_calibration.py: tool to calibrate HSV thresholds for different colours with sliders using opencv

# main.py: 
# specify the variables and constants: filter ratios for spike and odometry, paths for the different challenges / scenarios 
# get the initial position of the robot by checking the distance away from the walls
# merging the positions derived from odometry and spike using complementary filter 
# while loop that checks whether the path count has reached the limit to stop the programme automatically

# Miscellaneous / Testing
# motor wiring.avif: wiring for the encoder 
# button.py: activate the code when button pressed 
# Plotting using sockets
# client_raspi.py: functions for the raspberry pi to send the lidar points to the computer for plotting 
# server_comp.py: receive lidars points to plot, switch between cartesian or polar plot 
# main_plotting.py: for debugging socket code

# Uploading and debugging code:
# set up ssh in raspberry pi
# Log in using [username]@ipaddress, [password]
# Upload code by typing:
# scp C:\[file directory]\*.py [username]@ipaddres:/[file directory]
# (*.py uploads python files only as the github repository has non-python files)

# Laser cutting/ 3D printing files:
# Onshape CAD: Chassis, brackets, camera/servo mounts, etc.
# https://cad.onshape.com/documents?resourceType=resourceuserowner&nodeId=66495b1552a5e72f9db8f36a
# Files named “[Date] Design": contain the updates made to the chassis of the car before the final design. They also contain designs for the Ackermann steering linkages, which ensure that the inner front wheel steers at a larger angle than the outer front wheel when the car turns
# “Lidar measurements”: used for simpler visualisation of the first version of the initial_pose(), which calculates the car’s starting position and if it is on the left or right side of the map.
# “Pcb spacer”: 3d designs to ensure that the pcb remains balanced and does not tip over which could potentially affect connections between the pcb and raspberrypi
# “Camera mount”: 3d camera mounts which point the camera at a certain angle
# “Differential plate”: helps to lower the axle of the rear wheels to match the level of the motor
# “Lidar mount”: plate that secures the lidar sensor to the upper chassis of the car

