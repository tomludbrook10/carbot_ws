#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/tom/carbot_ws/install/setup.bash
exec ros2 launch carbot_ackermann run_carbot.launch.py
