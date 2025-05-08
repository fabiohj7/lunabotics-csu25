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
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='image_transmission',
            executable='image_transmission',
            name='image_transmission',
            output='screen'
        ),
        Node(
            package='crunch_tags',
            executable='crunch_tags',
            name='crunch_tags',
            output='screen'
        ),
        Node(
            package='autonomous',
            executable='autonomous',
            name='autonomous',
            output='screen'
        )
    ])
