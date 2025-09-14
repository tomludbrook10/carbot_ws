import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# ROS2 Humble compatible imports
try:
    # Try the newer import structure first (Iron/Jazzy)
    from ros_gz_bridge.actions import RosGzBridge
    from ros_gz_sim.actions import GzServer
    HAS_NEW_STRUCTURE = True
except ImportError:
    try:
        # Fall back to Humble structure
        from ros_gz_bridge import create_bridge_node
        from launch.actions import Node as LaunchNode
        HAS_NEW_STRUCTURE = False
        HAS_GAZEBO_PACKAGES = True
    except ImportError:
        HAS_GAZEBO_PACKAGES = False
        HAS_NEW_STRUCTURE = False

def generate_launch_description():
    pkg_share = get_package_share_directory('carbot_description')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'carbot_description.sdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    world_path = os.path.join(pkg_share, 'world', 'my_world.sdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, 
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Basic launch description
    launch_description_list = [
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        robot_state_publisher_node,
        rviz_node
    ]

    # For ROS2 Humble, use the older approach with direct nodes
    if not HAS_NEW_STRUCTURE:
        # Humble-style Gazebo launch
        gz_sim_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-r', world_path],
            output='screen'
        )
        
        # Humble-style bridge - manual topic bridging
        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
                '--ros-args', '-p', 'use_sim_time:=true'
            ],
            output='screen'
        )
        
        # Spawn entity for Humble
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', '/robot_description', '-name', 'carbot', '-z', '0.65'],
            output='screen'
        )

        launch_description_list.extend([
            ExecuteProcess(cmd=['ign', 'gazebo', '-r', world_path], output='screen'),
            bridge_node,
            spawn_entity,
        ])
    
    else:
        # Newer structure (Iron/Jazzy) - your original code works here
        ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
        gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")

        gz_server = GzServer(
            world_sdf_file=world_path,
            container_name='ros_gz_container',
            create_own_container='True',
            use_composition='True',
        )
        
        ros_gz_bridge = RosGzBridge(
            bridge_name='ros_gz_bridge',
            config_file=bridge_config_path,
            container_name='ros_gz_container',
            create_own_container='False',
            use_composition='True',
        )
        
        spawn_entity = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
            launch_arguments={
                'world': 'my_world',
                'topic': '/robot_description',
                'entity_name': 'carbot',
                'z': '0.65',
            }.items(),
        )

        launch_description_list.extend([
            ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
            gz_server,
            ros_gz_bridge,
            spawn_entity,
        ])

    return LaunchDescription(launch_description_list)