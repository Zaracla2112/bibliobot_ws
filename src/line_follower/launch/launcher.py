from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_follower',
            executable='serial_controller',
            name='serial_controller',
            output='screen'
        ),
        Node(
            package='line_follower',
            executable='prueba',
            name='prueba',
            output='screen'
        ),
    ])
