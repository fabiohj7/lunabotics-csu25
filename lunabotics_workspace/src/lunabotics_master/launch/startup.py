from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='networking',
            executable='networking',
            name='networking',
            output='screen'
        ),
        Node(
            package='robo_driver',
            executable='robo_driver',
            name='robo_driver',
            output='screen'
        ),
        Node(
            package='manual_drive',
            executable='manual_drive',
            name='manual_drive',
            output='screen'
        )
    ])
