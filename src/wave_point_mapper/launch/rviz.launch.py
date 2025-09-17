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

    pkg_dir = get_package_share_directory('wave_point_mapper')
    # RViz config file
    rviz_config = os.path.join(pkg_dir, 'config', 'waypoint_creator.rviz')
         # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )
    
    return LaunchDescription([
        # Arguments
        rviz_node
    ])