from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Gps to odometry conversion node
        Node(
            package='gps_conversion',
            executable='gps_to_odom',
            name='gps_to_odom',
            output='screen',
            parameters=[
                {'lat_r': 45.47769},
                {'lon_r': 9.226740},
                {'alt_r': 169.04100}
            ]
        ),

        # Odometry to tf conversion node
        Node(
            package='gps_conversion',
            executable='odom_to_tf',
            name='odom_to_tf_gps',
            output='screen',
            parameters=[
                {'root_frame': 'world'},
                {'child_frame': 'gps_odom'}
            ],
            remappings=[
                ('input_odom', 'gps_odom')
            ]
        ),

        # Rviz start
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen'
        )
    ])
