from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package path
    martian_mines_dir = get_package_share_directory('martian_mines')

    # Define launch arguments
    real_world_arg = DeclareLaunchArgument('real_world', default_value='false', description='Run in real-world mode')
    no_start_pose_arg = DeclareLaunchArgument('no_start_pose', default_value='false', description='Skip start pose')

    # Include core.launch.xml
    core_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([martian_mines_dir, 'launch', 'core.launch.xml'])),
        launch_arguments={'real_world': LaunchConfiguration('real_world'),
                          'no_start_pose': LaunchConfiguration('no_start_pose')}.items()
    )

    # Define UAV0 nodes
    uav0_nodes = [
        Node(
            package='martian_mines',
            executable='precision_landing.py',
            name='precision_landing',
            output='screen',
            parameters=[PathJoinSubstitution([martian_mines_dir, 'config', 'aruco.yaml'])]
        ),
        Node(
            package='martian_mines',
            executable='detection.py',
            name='detection',
            output='screen'
        ),
        Node(
            package='martian_mines',
            executable='bbox_publisher.py',
            name='land_on_target',
            output='screen'
        ),
    ]

    # Conditional remaps (in real-world mode)
    remap_real_world = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='remap_camera',
            arguments=['camera/image_raw', 'color/image_raw']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='remap_camera_info',
            arguments=['camera/camera_info', 'color/camera_info']
        )
    ]

    return LaunchDescription([
        real_world_arg,
        no_start_pose_arg,
        core_launch
    ] + uav0_nodes + remap_real_world if LaunchConfiguration('real_world') else uav0_nodes)
