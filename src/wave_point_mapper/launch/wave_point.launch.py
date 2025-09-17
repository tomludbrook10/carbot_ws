#!/usr/bin/env python3
"""
Launch file for map visualization and waypoint creation
Starts map server, RViz, and waypoint creator tool
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Parameters
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    waypoint_file = LaunchConfiguration('waypoint_file')


    
    # Get package directory
    pkg_dir = get_package_share_directory('wave_point_mapper')
    
    # RViz config file
    map_server_config = os.path.join(pkg_dir, 'config', 'map_server.yaml')
    
    # Map server
    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_file,
                'use_sim_time': use_sim_time}])

    map_server_lifecycle = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server']}])

    # Waypoint creator tool
    wave_pointer_creator = Node(
        package='wave_point_mapper',
        executable='wave_point_creator',
        name='wave_point_creator',
        output='screen',
        parameters=[{
            'waypoint_file': waypoint_file,
            'create_mid_section_path': True
        }]
    )

    # Static transform for map to odom (temporary, for visualization)
    static_transform =   Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join('/home/tom/carbot_ws', 'maps', 'maps.yaml'),
            description='Full path to map yaml file'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'waypoint_file',
            default_value=os.path.join('/home/tom/carbot_ws/src/wave_point_mapper', 'config', 'waypoints.yaml'),
            description='Path to save/load waypoints'
        ),
        map_server,
        map_server_lifecycle,
        wave_pointer_creator,
        static_transform
    ])