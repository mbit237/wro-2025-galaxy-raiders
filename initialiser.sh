#!/bin/sh
cd /home/joshua
if /home/joshua/main.py; then 
    echo open
    # ./main_open.py 
else; 
    echo obstacle
    # ./main_obstacle.py
fi
