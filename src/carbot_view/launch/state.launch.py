from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='carbot_view').find('carbot_view')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'carbot_description.urdf')
    
    static_transform_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_ldlidar_tf',
    arguments=[
        '0.040', '0.005', '0.090',  # x, y, z position
        '0', '0', '0',         # roll, pitch, yaw rotation
        'base_link',           # parent frame
        'ldlidar_base'         # child frame
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}])

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        robot_state_publisher_node,
        static_transform_node,
    ])