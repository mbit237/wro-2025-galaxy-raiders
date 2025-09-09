#!/bin/sh
cd /home/joshua
while true; do
echo doing
if /home/joshua/main.py; then 
    echo open > /home/joshua/fur.txt
    # ./main_open.py 
else 
    echo obstacle > /home/joshua/fur.txt
    # ./main_obstacle.py
fi
echo done
sleep 3
done
