#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction,
    RegisterEventHandler,
    EmitEvent,
    LogInfo
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    node_name = 'lidar_node'

    pkg_dir = get_package_share_directory('acml_with_map')
    map_file = LaunchConfiguration('map_file')
    amcl_config = os.path.join(pkg_dir, 'config', 'amcl.yaml')
    
    # Initial pose parameters
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')


    robot_odom_node  = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('carbot_ackermann'),
            '/launch/carbot_ackermann_launch.py'
        ]))

    # Robot state publisher (if testing)
    robot_state_node = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('carbot_view'),
            '/launch/state.launch.py'
        ])
    )

    # LDLidar with lifecycle manager
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_with_mgr.launch.py'
        ]),
        launch_arguments={
            'node_name': node_name
        }.items()
    )
    
    # Map server as regular node (not lifecycle)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'topic_name': 'map',
            'frame_id': 'map'
        }]
    )

    # Map server lifecycle manager
    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # AMCL as a lifecycle node
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        remappings=[
            ('scan', '/ldlidar_node/scan')  # Make sure this matches your LIDAR topic
        ],
        parameters=[amcl_config])

    # AMCL Lifecycle Manager - handles configuration and activation
    amcl_lifecycle_manager = Node(
        package='acml_with_map',
        executable='amcl_lifecycle_manager',
        name='amcl_lifecycle_manager',
        output='screen',
        parameters=[{
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
            'initial_cov_xx': 0.25,
            'initial_cov_yy': 0.25,
            'initial_cov_aa': 0.068,
            'wait_for_map': True,
            'auto_start': True,
            'set_initial_pose': True
        }]
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join('/home/tom/carbot_ws', 'maps', 'maps.yaml'),
            description='Full path to map yaml file'
        ),
        
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Initial pose X coordinate'
        ),
        
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Initial pose Y coordinate'
        ),
        
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Initial pose yaw angle (radians)'
        ),
        
        # Launch nodes in sequence
        # 1. First launch robot state and LIDAR
        robot_state_node,
        robot_odom_node,
        ldlidar_launch,
        
        # 2. Launch map server immediately
        map_server,
        map_server_lifecycle,
        
        # 3. Launch AMCL node (as lifecycle node)
        TimerAction(
            period=7.0,  # Wait for map server to start
            actions=[amcl_node]
        ),
        
        # 4. Launch AMCL lifecycle manager to configure and activate it
        TimerAction(
            period=10.0,  # Wait for AMCL node to be created
            actions=[amcl_lifecycle_manager]
        ),
    ])