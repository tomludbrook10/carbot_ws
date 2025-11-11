#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/tom/carbot_ws/install/setup.bash

data_dir="/home/tom/carbot_ws/model_rollouts/$(date +%s)"
mkdir -p "$data_dir"

exec ros2 launch carbot_inference carbot_pp_inf.launch.py debug_mode:=true debug_output_dir:="$data_dir" num_waypoints:=4 model_name:=outside_office
