#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/tom/carbot_ws/install/setup.bash
export GST_PLUGIN_PATH=/home/tom/carbot_teleoperation/src/gst-plugins
exec ros2 run teleop keyboard_teleop