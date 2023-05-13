from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sfr',
            executable='encoder_publisher_node',
            name='encoder_publisher_node'
        ),
        Node(
            package='sfr',
            executable='ultrasonic_publisher_node',
            name='ultrasonic_publisher_node'
        ),
        Node(
            package='sfr',
            executable='scd30_publisher_node',
            name='scd30_publisher_node'
        )
    ])
