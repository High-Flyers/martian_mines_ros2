import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([

        # Push the namespace 'uav0' before launching the nodes
        launch_ros.actions.PushRosNamespace('uav0'),

            launch_ros.actions.Node(
                package='martian_mines',
                executable='detection_visualization',
                name='detections_visualization',
                output='screen'
            ),
            launch_ros.actions.Node(
                package='martian_mines',
                executable='detection_visualization',
                name='confirmed_figures_visualization',
                output='screen',
                parameters=[{'marker_size': 0.5}],
                remappings=[('figure_finder/detected_figures', 'figure_finder/confirmed_figures')]
            )
        ])
    
