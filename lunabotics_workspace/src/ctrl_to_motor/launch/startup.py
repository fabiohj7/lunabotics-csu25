from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='ctrl_to_motor',
            executable='ctrl_to_motor',
            name='ctrl_to_motor',
            output='screen'
        )
    ])
