import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions


def generate_launch_description():
    config = DeclareLaunchArgument(
        "config", 
        default_value=os.path.join(get_package_share_directory("martian_mines"), "config", "mission.rviz")
    )

    namespace = launch_ros.actions.PushRosNamespace('uav0')

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", LaunchConfiguration("config")],
        output="screen"
    )

    environment_visualization = Node(
        package="martian_mines",
        executable="environment_visualization",
        name="environment_visualization",
        output="screen"
    )

    return launch.LaunchDescription([
        namespace,
        config,
        rviz,
        environment_visualization
    ])
