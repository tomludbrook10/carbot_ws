#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/tom/carbot_ws/install/setup.bash
exec ros2 run carbot_ackermann serial_manager_node
