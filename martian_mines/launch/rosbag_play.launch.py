import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    real_world_arg = DeclareLaunchArgument("real_world", default_value="true")
    bag_file_arg = DeclareLaunchArgument("bag_file", default_value="")
    no_start_pose_arg = DeclareLaunchArgument("no_start_pose", default_value="false")
    
    config_file = LaunchConfiguration("config_file")
    real_world = LaunchConfiguration("real_world")
    bag_file = LaunchConfiguration("bag_file")
    
    set_sim_time = SetParameter("use_sim_time", value=True)
    
    set_config_file = SetLaunchConfiguration(
        "config_file", 
        PathJoinSubstitution([
            get_package_share_directory("martian_mines"),
            "config",
            "real.yaml"
        ]) if real_world else PathJoinSubstitution([
            get_package_share_directory("martian_mines"),
            "config",
            "sim.yaml"
        ])
    )
    
    player_node = ExecuteProcess(
    cmd=[
        FindExecutable(name="ros2"),
        "bag", "play",
        PathJoinSubstitution([
            get_package_share_directory("martian_mines"), "data", bag_file
        ]),
        "--clock", "-l"
    ],
    output="screen"
    )

    
    uav0_group = GroupAction([
        Node(
            package="martian_mines",
            executable="detection",
            name="detection",
            output="screen"
        ),
        Node(
            package="martian_mines",
            executable="figure_finder",
            name="figure_finder",
            output="screen",
            parameters=[{"config_file_path": config_file}]
        ),
        ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "service", "call",
            "figure_finder/start",
            "std_srvs/srv/Trigger"
            ],
        output="screen"
    )
    ])
    
    return LaunchDescription([
        real_world_arg,
        bag_file_arg,
        no_start_pose_arg,
        set_sim_time,
        set_config_file,
        player_node,
        uav0_group
    ])
