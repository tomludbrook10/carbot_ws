# launch/ackermann_robot.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Find package directory
    pkg_share = FindPackageShare('carbot_ackermann')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'carbot_ackermann_params.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Serial Manager Node - MUST start successfully
    serial_manager_node = Node(
        package='carbot_ackermann',
        executable='serial_manager_node',
        name='serial_manager',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        on_exit=Shutdown(),  # If serial manager fails, shutdown everything
        respawn=False,
        emulate_tty=True
    )
    
    # Odometry Node - starts after Serial Manager
    odometry_node = Node(
        package='carbot_ackermann',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        respawn=True,
        respawn_delay=2.0
    )
    
    # Control Node
    control_node = Node(
        package='carbot_ackermann',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        respawn=True,
        respawn_delay=2.0
    )
    
    # Log startup info
    startup_info = LogInfo(
        msg='Starting Ackermann Robot System - Serial Manager MUST connect to ESP32 or system will shut down'
    )
    
    return LaunchDescription([
        # Launch arguments
        config_file_arg,
        use_sim_time_arg,
        
        # Log message
        startup_info,
        
        # Nodes - Serial Manager starts first and must succeed
        serial_manager_node,
        odometry_node,
        control_node
    ])