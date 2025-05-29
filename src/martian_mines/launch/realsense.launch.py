import os
import launch
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


def generate_launch_description():

    realsense_launch_path = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch",
        "rs_launch.py"
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
    )

    uav0 = GroupAction([realsense_launch])

    return launch.LaunchDescription([
        uav0
    ])