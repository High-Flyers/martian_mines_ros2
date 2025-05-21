from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
import launch_ros.actions

def generate_launch_description():
    real_world_arg = DeclareLaunchArgument("real_world", default_value="false")
    no_start_pose_arg = DeclareLaunchArgument("no_start_pose", default_value="false")

    real_world = LaunchConfiguration("real_world")
    no_start_pose = LaunchConfiguration("no_start_pose")
    
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("martian_mines"), "launch", "core.launch.py"
            ])
        ),
        launch_arguments={"real_world": real_world, "no_start_pose": no_start_pose}.items()
    )

    uav0_group = GroupAction([
        Node(
            package="martian_mines",
            executable="detection",
            name="detection",
            output="screen",
            #### TO DO ####
            #### remap condition if real world == "true"
            #remappings=[("/camera/image_raw", "/color/image_raw"),
            #("/camera/camera_info", "/color/camera_info")]
        ),
        Node(
            package="martian_mines",
            executable="figure_finder",
            name="figure_finder",
            output="screen"
        ),
        Node(
            package="martian_mines",
            executable="report_uploader",
            name="uploader",
            output="screen"
        ),
        ExecuteProcess(
            cmd=[FindExecutable(name="ros2"), "service", "call", "/figure_finder/start", "std_srvs/srv/Trigger"],
            output="screen"
        )
    ])

    return LaunchDescription([
        real_world_arg,
        no_start_pose_arg,
        core_launch,
        uav0_group
    ])
