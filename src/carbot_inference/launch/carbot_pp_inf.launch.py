# launch/ackermann_robot.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, EmitEvent
from launch_ros.actions import Node

def generate_launch_description():
    
    waypoint_sender_node = Node(
        package='carbot_inference',
        executable='waypoint_sender',
        name='waypoint_sender_node',
        output='screen'
    )
    
    pure_pursuit_controller_node = Node(
        package='carbot_inference',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller_node',
        output='screen'
    )


    return LaunchDescription([
        waypoint_sender_node,
        pure_pursuit_controller_node
    ])