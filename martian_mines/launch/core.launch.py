import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    martian_mines_share = get_package_share_directory('martian_mines')
    mavros_share = get_package_share_directory('mavros')

    return LaunchDescription([
        # Set PX4 simulation speed factor
        SetEnvironmentVariable('PX4_SIM_SPEED_FACTOR', '1'),

        # Declare launch arguments
        DeclareLaunchArgument('real_world', default_value='false', description='Use real-world configuration'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(martian_mines_share, 'config', 'sim.yaml')),
        DeclareLaunchArgument('no_start_pose', default_value='false', description='Disable start pose transform'),

        # Define config file based on real_world argument
        GroupAction(
            condition=IfCondition(LaunchConfiguration('real_world')),
            actions=[
                DeclareLaunchArgument('config_file', default_value=os.path.join(martian_mines_share, 'config', 'real.yaml')),
            ]
        ),

        # Real-world setup
        GroupAction(
            condition=IfCondition(LaunchConfiguration('real_world')),
            actions=[
                # Include RealSense launch
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(os.path.join(martian_mines_share, 'launch', 'realsense.launch.xml'))
                ),
                # Include MAVROS launch
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(os.path.join(mavros_share, 'launch', 'px4.launch.xml')),
                    launch_arguments={'fcu_url': 'udp://:14550@localhost:14550'}.items(),
                ),
                # Set MAVROS parameters
                Node(
                    package='ros2param', executable='ros2param', output='screen',
                    arguments=['load', os.path.join(martian_mines_share, 'launch', 'mavros_plugins.yaml')]
                ),
                # Static TF for camera
                Node(
                    package='tf2_ros', executable='static_transform_publisher',
                    name='tf_base_link_camera',
                    arguments=['0.18', '0', '-0.1', '1.571', '3.14', '0', 'base_link', 'camera_link']
                ),
            ]
        ),

        # Simulation setup
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('real_world')),
            actions=[
                # Static TF for camera
                Node(
                    package='tf2_ros', executable='static_transform_publisher',
                    name='tf_base_link_camera',
                    arguments=['0', '0', '0', '-1.571', '0', '-1.571', 'cgo3_camera_link', 'camera_link']
                ),
                # Static TF for development
                Node(
                    package='tf2_ros', executable='static_transform_publisher',
                    name='tf_uber_map',
                    arguments=['30.5', '-18.5', '0', '0', '0', '0', 'map', 'uber_map']
                ),
            ]
        ),

        # UAV0 namespace setup
        GroupAction(
            actions=[
                # Load configuration file
                Node(
                    package='ros2param', executable='ros2param', output='screen',
                    arguments=['load', LaunchConfiguration('config_file')]
                ),
                # Topic remapping for real-world scenario
                Node(
                    condition=IfCondition(LaunchConfiguration('real_world')),
                    package='ros2topic', executable='ros2topic',
                    arguments=['remap', 'camera/image_raw:=color/image_raw']
                ),
                Node(
                    condition=IfCondition(LaunchConfiguration('real_world')),
                    package='ros2topic', executable='ros2topic',
                    arguments=['remap', 'camera/camera_info:=color/camera_info']
                ),
                # Start pose transform node
                Node(
                    condition=UnlessCondition(LaunchConfiguration('no_start_pose')),
                    package='martian_mines', executable='tf_start_pose.py',
                    name='tf_start_pose', output='screen'
                ),
            ]
        )
    ])
