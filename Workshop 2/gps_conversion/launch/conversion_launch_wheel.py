from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo gps_to_odom
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

        # Nodo odom_to_tf per wheel_odom
        Node(
            package='gps_conversion',
            executable='odom_to_tf',
            name='odom_to_tf_wheel',
            output='screen',
            parameters=[
                {'root_frame': 'world'},
                {'child_frame': 'wheel_odom'}
            ],
            remappings=[
                ('input_odom', 'odom')
            ]
        ),

        # Nodo odom_to_tf per gps_odom
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

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_wheel',
            arguments=[
            '0.01393',    # x
            '-2.282713',  # y
            '0.0',        # z
            '0.0',        # roll
            '0.0',        # pitch
            '3.1416',     # yaw (180°)
                'world',      # parent frame
                'wheel_odom'  # child frame
            ],
            output='screen'
        ),


        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen'
        )
    ])
