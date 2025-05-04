# File: test_launch.launch.py

import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription([
        # Avvia il primo nodo (pub)
        launch_ros.actions.Node(
            package='package_prova',
            executable='Test_pub',
            name='Pub',
            output='screen'
        ),
        # Avvia il secondo nodo (sub)
        launch_ros.actions.Node(
            package='package_prova',
            executable='Test_sub',
            name='Sub',
            output='screen',

        )
    ])