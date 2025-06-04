from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package path
    martian_mines_dir = get_package_share_directory('martian_mines')

    # Define launch arguments
    real_world_arg = DeclareLaunchArgument('real_world', default_value='false', description='Run in real-world mode')
    no_start_pose_arg = DeclareLaunchArgument('no_start_pose', default_value='false', description='Skip start pose')

    # Include core.launch.xml
    core_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([martian_mines_dir, 'launch', 'core.launch.py'])),
        launch_arguments={'real_world': LaunchConfiguration('real_world'),
                          'no_start_pose': LaunchConfiguration('no_start_pose')}.items()
    )

    # Define UAV0 nodes
    uav0_nodes = GroupAction([
        # launch_ros.actions.PushRosNamespace('uav0'),
        Node(
            package='martian_mines',
            executable='precision_landing',
            name='precision_landing',
            output='screen',
            parameters=[
                PathJoinSubstitution([martian_mines_dir, 'config', 'aruco.yaml']),
                {"use_sim_time": True},
            ],
        ),
        Node(
            package='martian_mines',
            executable='detection',
            name='detection',
            output='screen',
            parameters=[
                {"detector": "aruco"},
                {"use_sim_time": True},
            ],
        ),
        Node(
            package='martian_mines',
            executable='bbox_publisher',
            name='land_on_target',
            output='screen',
            parameters=[
                {"use_sim_time": True},
            ],
        ),
        launch_ros.actions.Node(
            package="martian_mines", executable="mission_controller", output="screen", parameters=[{'use_sim_time': True}]
        ),
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", PathJoinSubstitution([martian_mines_dir, 'config', 'precision_landing.rviz'])],
            output="screen",
        ),
    ])

    return LaunchDescription([
        real_world_arg,
        no_start_pose_arg,
        core_launch,
        uav0_nodes
    ])
