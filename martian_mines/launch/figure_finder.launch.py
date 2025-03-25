from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import launch_ros
from launch_ros.actions import Node

def generate_launch_description():
    # Get package path
    martian_mines_dir = get_package_share_directory('martian_mines')

    # Define launch arguments
    real_world_arg = DeclareLaunchArgument('real_world', default_value='false', description='Run in real-world mode')
    no_start_pose_arg = DeclareLaunchArgument('no_start_pose', default_value='false', description='Skip start pose')

    # Include core.launch.xml
    # core_launch = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(PathJoinSubstitution([martian_mines_dir, 'launch', 'core.launch.xml'])),
    #     launch_arguments={'real_world': LaunchConfiguration('real_world'),
    #                       'no_start_pose': LaunchConfiguration('no_start_pose')}.items()
    # )

    uav0_nodes = [
        Node(
            package='martian_mines',
            executable='figure_finder',
            name='figure_finder',
            output='screen',
            parameters=[PathJoinSubstitution()]
        ),
        Node(
            package='martian_mines',
            executable='detection',
            name='detection',
            output='screen',
            # parameters=[PathJoinSubstitution([martian_mines_dir, 'config', 'aruco.yaml'])]
        ),
        Node(
            package='martian_mines',
            executable='report_uploader',
            name='report_uploader',
            output='screen',
            # parameters=[PathJoinSubstitution([martian_mines_dir, 'config', 'aruco.yaml'])]
        )
    ]

    # Conditional remaps (in real-world mode)
    # remap_real_world = [
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='remap_camera',
    #         arguments=['camera/image_raw', 'color/image_raw']
    #     ),
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='remap_camera_info',
    #         arguments=['camera/camera_info', 'color/camera_info']
    #     )
    # ]



    argstlist =[]
    argstlist.append(launch_ros.actions.PushRosNamespace('uav0'))
    # argstlist.append(core_launch)

    argstlist=argstlist+uav0_nodes
    # argstlist=argstlist+remap_real_world
    argstlist.append(real_world_arg)
    argstlist.append(no_start_pose_arg)


    # print(argstlist)
    return LaunchDescription(
        argstlist
    )

