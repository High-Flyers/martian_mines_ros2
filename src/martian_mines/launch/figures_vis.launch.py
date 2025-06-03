import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
            launch_ros.actions.Node(
                package='martian_mines',
                executable='detection_visualization',
                name='detections_visualization',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),
            launch_ros.actions.Node(
                package='martian_mines',
                executable='detection_visualization',
                name='confirmed_figures_visualization',
                output='screen',
                parameters=[{'marker_size': 0.5},{'use_sim_time': True}],
                remappings=[('figure_finder/detected_figures', 'figure_finder/confirmed_figures')]
            )
        ])
    
