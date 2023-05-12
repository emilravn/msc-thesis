#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32


class Subscriber(Node):
    def __init__(self):
        super().__init__("motor_subscriber")  # type: ignore

        self.back_ultrasonic_subscription = self.create_subscription(
            Range, "ultrasonic/back/distance", self.back_ultrasonic_listener_callback, 10
        )
        self.back_ultrasonic_subscription

        self.middle_ultrasonic_subscription = self.create_subscription(
            Range, "ultrasonic/middle/distance", self.middle_ultrasonic_listener_callback, 10
        )
        self.middle_ultrasonic_subscription

        self.front_ultrasonic_subscription = self.create_subscription(
            Range, "ultrasonic/front/distance", self.front_ultrasonic_listener_callback, 10
        )
        self.front_ultrasonic_subscription

        self.back_ultrasonic_distance = None
        self.middle_ultrasonic_distance = None
        self.front_ultrasonic_distance = None

        self.right_encoder_subscription = self.create_subscription(
            Float32, "right_encoder/distance", self.right_encoder_listener_callback, 10
        )
        self.right_encoder_subscription

        self.left_encoder_subscription = self.create_subscription(
            Float32, "left_encoder/distance", self.left_encoder_listener_callback, 10
        )
        self.left_encoder_subscription

        self.total_encoder_subscription = self.create_subscription(
            Float32, "total_encoder/distance", self.total_encoder_listener_callback, 10
        )
        self.total_encoder_subscription

    def back_ultrasonic_listener_callback(self, msg: Range):
        self.get_logger().info(f"back distance: {msg.range}")
        self.back_ultrasonic_distance = msg.range

    def middle_ultrasonic_listener_callback(self, msg: Range):
        self.get_logger().info(f"middle distance: {msg.range}")
        self.middle_ultrasonic_distance = msg.range

    def front_ultrasonic_listener_callback(self, msg: Range):
        self.get_logger().info(f"front distance: {msg.range}")
        self.front_ultrasonic_distance = msg.range

    def right_encoder_listener_callback(self, msg: Float32):
        self.get_logger().info(f"right encoder distance: {msg.data}")
        self.right_encoder_distance = msg.data

    def left_encoder_listener_callback(self, msg: Float32):
        self.get_logger().info(f"left encoder distance: {msg.data}")
        self.left_encoder_distance = msg.data

    def total_encoder_listener_callback(self, msg: Float32):
        self.get_logger().info(f"total encoder distance: {msg.data}")
        self.total_encoder_distance = msg.data


def main(args=None):
    rclpy.init(args=args)

    motor_subscriber = Subscriber()
    rclpy.spin(motor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
