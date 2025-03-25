import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    real_world = DeclareLaunchArgument("real_world", default_value="false")
    no_start_pose = DeclareLaunchArgument("no_start_pose", default_value="false")
    plot_trajectory = DeclareLaunchArgument("plot_trajectory", default_value="false")

    core_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("martian_mines"), "launch", "core.launch.py"),
        launch_arguments={
            "real_world": LaunchConfiguration("real_world"),
            "no_start_pose": LaunchConfiguration("no_start_pose")
        }.items()
    )

    uav0 = GroupAction([
        Node(
            package="martian_mines",
            executable="trajectory_generator.py",
            name="trajectory_generator",
            output="screen",
            parameters=[{"plot": LaunchConfiguration("plot_trajectory")}]
        ),
        Node(
            package="martian_mines",
            executable="trajectory_tracker.py",
            name="trajectory_tracker",
            output="screen",
            remappings=[("trajectory_tracker/path", "trajectory_generator/path")]
        ),
        ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "service", "call", "--wait",
            "/trajectory_generator/generate",
            "std_srvs/srv/Trigger"
        ],
        output="screen"
    )
    ])
    
    return launch.LaunchDescription([
        real_world,
        no_start_pose,
        plot_trajectory,
        core_launch,
        uav0
    ])
