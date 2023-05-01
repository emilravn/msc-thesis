from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reactive_robot',
            executable='encoder_publisher',
            name='encoder_publisher_node'
        ),
        Node(
            package='reactive_robot',
            executable='ultrasonic_publisher',
            name='ultrasonic_publisher_node'
        )
    ])
