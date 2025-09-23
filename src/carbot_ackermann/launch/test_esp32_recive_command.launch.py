from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': '/home/tom/carbot_ws/maps/maps.yaml',
                        'use_sim_time': False}],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            parameters=[{'autostart': True,
                        'node_names': ['map_server']}],
            output='screen'
        )
    ])