import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    martian_mines_share = get_package_share_directory('martian_mines')

    real_world = LaunchConfiguration('real_world')
    config_file = LaunchConfiguration('config_file')
    no_start_pose = LaunchConfiguration('no_start_pose')


    return LaunchDescription([
        # PX4 Environment variable
        SetEnvironmentVariable('PX4_SIM_SPEED_FACTOR', '1'),

        # Declare arguments
        DeclareLaunchArgument('real_world', default_value='false'),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(martian_mines_share, 'config', 'sim.yaml'),
            condition=UnlessCondition(real_world)
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(martian_mines_share, 'config', 'real.yaml'),
            condition=IfCondition(real_world)
        ),
        DeclareLaunchArgument('no_start_pose', default_value='false'),

        # Real-world setup
        GroupAction(
            condition=IfCondition(real_world),
            actions=[
                # Include realsense camera launch
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(martian_mines_share, 'launch', 'realsense.launch.py')
                    )
                ),
                # Static transform for camera
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='tf_base_link_camera',
                    arguments=['0.18', '0', '-0.1', '1.571', '3.14', '0', 'base_link', 'camera_link']
                )
            ]
        ),

        # Simulation-only setup
        GroupAction(
            condition=UnlessCondition(real_world),
            actions=[
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='tf_base_link_camera',
                    arguments=['0', '0', '0', '-1.571', '0', '-1.571', 'cgo3_camera_link', 'camera_link']
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='tf_uber_map',
                    arguments=['30.5', '-18.5', '0', '0', '0', '0', 'map', 'uber_map']
                )
            ]
        ),

            # UAV0 namespace group
        GroupAction(
            actions=[
                # Figure Finder node
                Node(
                    package='martian_mines',
                    executable='figure_finder',
                    name='figure_finder',
                    output='screen',
                    parameters=[config_file]
                ),
                # Detection node
                Node(
                    package='martian_mines',
                    executable='detection',
                    name='detection',
                    output='screen',
                    parameters=[config_file]
                ),
                # tf_start_pose node
                Node(
                    condition=UnlessCondition(no_start_pose),
                    package='martian_mines',
                    executable='tf_start_pose',
                    name='tf_start_pose',
                    output='screen',
                    parameters=[config_file]
                ),

                # Real-world remaps
                Node(
                    condition=IfCondition(real_world),
                    package='martian_mines',
                    executable='remap_helper.py',  # dummy placeholder if you want to do remapping
                    name='remap_camera_topics',
                    remappings=[
                        ('camera/image_raw', 'color/image_raw'),
                        ('camera/camera_info', 'color/camera_info')
                    ],
                    output='screen'
                )

            ]
        )
    ])
