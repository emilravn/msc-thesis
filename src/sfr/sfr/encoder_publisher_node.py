#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from .encoder_driver import Encoders


# Left encoder pins
LEFT_ENCODER_A = 26
LEFT_ENCODER_B = 16

# Right encoder pins
RIGHT_ENCODER_A = 25
RIGHT_ENCODER_B = 22


class EncoderPublisher(Node):
    def __init__(self, debug=False):
        super().__init__("encoder_publisher")

        self.encoders = Encoders(LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B)

        self.debug = debug

        self.left_encoder_publisher = self.create_publisher(Float32, "left_encoder/distance", 10)

        self.right_encoder_publisher = self.create_publisher(Float32, "right_encoder/distance", 10)

        self.total_encoder_publisher = self.create_publisher(Float32, "total_encoder/distance", 10)

        timer_seconds = 0.01

        self.left_encoder_timer = self.create_timer(timer_seconds, self.left_encoder_callback)

        self.right_encoder_timer = self.create_timer(timer_seconds, self.right_encoder_callback)

        self.total_encoder_timer = self.create_timer(timer_seconds, self.total_encoder_callback)

    def left_encoder_callback(self):
        msg = Float32()
        msg.data = self.encoders.get_distance_travelled_left()
        if self.debug:
            self.get_logger().info(f"left encoder publishing: {msg.data}mm")
        self.left_encoder_publisher.publish(msg)

    def right_encoder_callback(self):
        msg = Float32()
        msg.data = self.encoders.get_distance_travelled_right()
        if self.debug:
            self.get_logger().info(f"right encoder publishing: {msg.data}mm")
        self.right_encoder_publisher.publish(msg)

    def total_encoder_callback(self):
        msg = Float32()
        msg.data = self.encoders.get_total_distance_travelled()
        if self.debug:
            self.get_logger().info(f"total encoder publishing: {msg.data}mm")
        self.total_encoder_publisher.publish(msg)

    def stop_gpio_pins(self):
        self.encoders.clean_gpio()


def main(args=None):
    rclpy.init(args=args)
    encoder_publisher = EncoderPublisher()
    rclpy.spin(encoder_publisher)
    encoder_publisher.stop_gpio_pins()
    encoder_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
