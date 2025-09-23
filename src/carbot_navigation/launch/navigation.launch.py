#!/usr/bin/env python3
"""
Complete Navigation Launch File
Integrates AMCL localization, Nav2 stack, waypoint following, and robot control
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('carbot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Launch configurations
    map_file = LaunchConfiguration('map_file')
    waypoint_file = LaunchConfiguration('waypoint_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    auto_start = LaunchConfiguration('auto_start')
    use_follow_waypoints = LaunchConfiguration('use_follow_waypoints')
    
    # Default file paths
    default_map = os.path.join('/home/tom/carbot_ws', 'maps', 'maps.yaml')
    default_waypoints = os.path.join('/home/tom/carbot_ws', 'waypoints', 'waypoints.yaml')
    default_nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    return LaunchDescription([
        # ========== Launch Arguments ==========
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description='Full path to map YAML file'
        ),
        
        DeclareLaunchArgument(
            'waypoint_file',
            default_value=default_waypoints,
            description='Full path to waypoints YAML file'
        ),
        
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params,
            description='Full path to Nav2 parameters file'
        ),
        
        DeclareLaunchArgument(
            'auto_start',
            default_value='false',
            description='Auto start navigation after launch'
        ),
        
        DeclareLaunchArgument(
            'use_follow_waypoints',
            default_value='true',
            description='Use FollowWaypoints action (true) or NavigateToPose (false)'
        ),
        
        # ========== Nav2 Stack ==========
        TimerAction(
            period=0.0,  # Wait for localization to be ready
            actions=[
                LogInfo(msg='Starting Nav2 stack...'),
                
                # Controller Server
                LifecycleNode(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    namespace='',
                    output='screen',
                    parameters=[nav2_params_file],
                    remappings=[('cmd_vel', 'cmd_vel')]
                ),
                
                # Planner Server
                LifecycleNode(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    namespace='',
                    parameters=[nav2_params_file]
                ),
                
                # Recoveries Server
                LifecycleNode(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    namespace='',
                    parameters=[nav2_params_file]
                ),
                
                # Behavior Tree Navigator
                LifecycleNode(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    namespace='',
                    parameters=[nav2_params_file]
                ),
                
                # Waypoint Follower
                LifecycleNode(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                    namespace='',
                    parameters=[nav2_params_file]
                ),
                
                # Lifecycle Manager for Nav2
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'autostart': True,
                        'node_names': [
                            'controller_server',
                            'planner_server',
                            'behavior_server',
                            'bt_navigator',
                            'waypoint_follower'
                        ]
                    }]
                )
            ]
        ),
        

        # ========== Waypoint Navigation ==========
        TimerAction(
            period=20.0,  # Wait for Nav2 to be fully ready
            actions=[
                LogInfo(msg='Starting waypoint sender...'),
                Node(
                    package='carbot_navigation',
                    executable='waypoint_sender',
                    name='waypoint_sender',
                    output='screen',
                    parameters=[{
                        'waypoint_file': waypoint_file,
                        'loop_waypoints': False,
                        'use_follow_waypoints': use_follow_waypoints,
                        'auto_start': auto_start,
                        'waypoint_reached_tolerance': 0.3
                    }]
                )
            ]
        )
    ])
           