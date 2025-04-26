import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    martian_mines_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('martian_mines') + '/launch/realsense.launch.py'
        )
    )
    
    static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link_camera',
        arguments=['0.18', '0', '-0.1', '1.571', '3.14', '0', 'base_link', 'camera_link']
    )
    rosbag_path = get_package_share_directory('martian_mines') + '/data/'

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', rosbag_path, '-a'],
        output='screen',
    )

    return LaunchDescription([
        martian_mines_launch,
        static_tf,
        rosbag_record
    ])