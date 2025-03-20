import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    real_world = LaunchConfiguration("real_world")
    no_start_pose = LaunchConfiguration("no_start_pose")

    core_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory("martian_mines"), "launch", "core.launch.xml"
            ])
        ),
        launch_arguments={
            "real_world": real_world,
            "no_start_pose": no_start_pose
        }.items()
    )

    uav0_group = GroupAction([
        launch.actions.DeclareLaunchArgument("real_world", default_value="false"),
        launch.actions.DeclareLaunchArgument("no_start_pose", default_value="false"),

        launch_ros.actions.Node(
            package="martian_mines", executable="precision_landing.py", name="precision_landing", output="screen"
        ),
        launch_ros.actions.Node(
            package="martian_mines", executable="trajectory_generator.py", name="trajectory_generator", output="screen"
        ),
        launch_ros.actions.Node(
            package="martian_mines", executable="trajectory_tracker.py", name="trajectory_tracker", output="screen",
            remappings=[("trajectory_tracker/path", "trajectory_generator/path")]
        ),
        launch_ros.actions.Node(
            package="martian_mines", executable="detection.py", name="detection", output="screen"
        ),
        launch_ros.actions.Node(
            package="martian_mines", executable="figure_finder.py", name="figure_finder", output="screen"
        ),
        launch_ros.actions.Node(
            package="martian_mines", executable="report_uploader.py", name="report_uploader", output="screen"
        ),
        launch_ros.actions.Node(
            package="martian_mines", executable="mission_controller.py", name="mission_controller", output="screen"
        )
    ])

    figures_vis_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory("martian_mines"), "launch", "figures_vis.launch.xml"
            ])
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("real_world", default_value="false"),
        DeclareLaunchArgument("no_start_pose", default_value="false"),
        core_launch,
        uav0_group,
        figures_vis_launch
    ])
