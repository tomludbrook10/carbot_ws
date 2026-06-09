# launch/ackermann_robot.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    debug_output_dir = DeclareLaunchArgument(
        "debug_output_dir", default_value="",
        description="Directory to save debug outputs"
    )

    debug_mode = DeclareLaunchArgument(
        "debug_mode", default_value="false",
        description="Enable debug mode"
    )

    num_waypoints = DeclareLaunchArgument(
        "num_waypoints", default_value="8",
        description="Number of waypoints"
    )

    model_name = DeclareLaunchArgument(
        "model_name", default_value="simple_traj",
        description="Model name"
    )
    
    waypoint_sender_node = Node(
        package='carbot_inference',
        executable='waypoint_sender',
        name='waypoint_sender_node',
        output='screen',
        parameters=[{'debug_output_dir': LaunchConfiguration('debug_output_dir'), 
                     'debug_mode': LaunchConfiguration('debug_mode'), 
                     'num_waypoints': LaunchConfiguration('num_waypoints')},
                     {'model_name': LaunchConfiguration('model_name')}]
    )
    
    pure_pursuit_controller_node = Node(
        package='carbot_inference',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller_node',
        output='screen',
        parameters=[{'debug_output_dir': LaunchConfiguration('debug_output_dir'), 
                     'debug_mode': LaunchConfiguration('debug_mode'),
                     'num_waypoints': LaunchConfiguration('num_waypoints')}]
    )

    return LaunchDescription([
        debug_output_dir,
        debug_mode,
        num_waypoints,
        model_name,
        waypoint_sender_node,
        pure_pursuit_controller_node
    ])