import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("martian_mines"), "launch", "core.launch.py"
            ])
        ),
        launch_arguments={
            "real_world": LaunchConfiguration("real_world"),
            "no_start_pose": LaunchConfiguration("no_start_pose")
        }.items()
    )

    uav_group = GroupAction([
        launch.actions.DeclareLaunchArgument("real_world", default_value="false"),
        launch.actions.DeclareLaunchArgument("no_start_pose", default_value="false"),


        # launch_ros.actions.Node(
        #     package="martian_mines", executable="precision_landing", output="screen"
        # ),
        launch_ros.actions.Node(
            package="martian_mines", executable="trajectory_generator", output="screen", 
        ),
        # launch_ros.actions.Node(
        #     package="martian_mines", executable="report_uploader", output="screen"
        # ),
        launch_ros.actions.Node(
            package="martian_mines", executable="mission_controller", output="screen"
        ),
    ])

    figures_vis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("martian_mines"), "launch", "figures_vis.launch.py"
            ])
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("real_world", default_value="false"),
        DeclareLaunchArgument("no_start_pose", default_value="false"),
        core_launch,
        uav_group,
        figures_vis_launch
    ])
