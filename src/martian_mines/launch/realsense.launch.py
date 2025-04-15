import os
import launch
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


def generate_launch_description():
    serial_no = DeclareLaunchArgument("serial_no", default_value="")
    json_file_path = DeclareLaunchArgument("json_file_path", default_value="")

    realsense_launch_path = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch",
        "includes",
        "nodelet.launch.xml"
    )

    realsense_launch = IncludeLaunchDescription(
        launch_ros.actions.PushRosNamespace("camera"),
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            "serial_no": LaunchConfiguration("serial_no"),
            "json_file_path": LaunchConfiguration("json_file_path"),
            "color_width": "1280",
            "color_height": "720",
            "color_fps": "15",
            "enable_depth": "false",
            "enable_color": "true",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_fisheye": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "enable_pointcloud": "false",
            "enable_sync": "true",
            "tf_prefix": "camera"
        }.items()
    )

    uav0 = GroupAction([realsense_launch])

    return launch.LaunchDescription([
        serial_no,
        json_file_path,
        uav0
    ])