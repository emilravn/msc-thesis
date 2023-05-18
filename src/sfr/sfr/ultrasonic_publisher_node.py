#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from .ultrasonic_driver import DistanceSensorImpl


class UltrasonicPublisher(Node):
    def __init__(self, debug=False):
        super().__init__("ultrasonic_publisher")

        self.debug = debug

        self.back_sensor = DistanceSensorImpl("rear", 7, 8)
        self.middle_sensor = DistanceSensorImpl("middle", 27, 17)
        self.front_sensor = DistanceSensorImpl("front", 15, 14)

        self.back_ultrasonic_publisher_ = self.create_publisher(
            Range, "ultrasonic/back/distance", 10
        )
        self.middle_ultrasonic_publisher_ = self.create_publisher(
            Range, "ultrasonic/middle/distance", 10
        )
        self.front_ultrasonic_publisher_ = self.create_publisher(
            Range, "ultrasonic/front/distance", 10
        )

        timer_seconds = 0.01

        # timers
        self.back_ultrasonic_publisher_timer = self.create_timer(
            timer_seconds, self.back_sensor_callback
        )
        self.middle_ultrasonic_publisher_timer = self.create_timer(
            timer_seconds, self.middle_sensor_callback
        )
        self.front_ultrasonic_publisher_timer = self.create_timer(
            timer_seconds, self.front_sensor_callback
        )

    def create_range_msg(self, sensor: DistanceSensorImpl):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/sonar_link"
        msg.radiation_type = 0
        msg.field_of_view = 0.0
        msg.min_range = 0.0
        msg.max_range = 10.0
        msg.range = sensor.get_distance()
        return msg

    def back_sensor_callback(self):
        msg = self.create_range_msg(self.back_sensor)
        if self.debug:
            self.get_logger().info(f"{self.middle_sensor.location} sensor publishing: {msg.range}")
        self.back_ultrasonic_publisher_.publish(msg)

    def middle_sensor_callback(self):
        msg = self.create_range_msg(self.middle_sensor)
        if self.debug:
            self.get_logger().info(f"{self.middle_sensor.location} sensor publishing: {msg.range}")
        self.middle_ultrasonic_publisher_.publish(msg)

    def front_sensor_callback(self):
        msg = self.create_range_msg(self.front_sensor)
        if self.debug:
            self.get_logger().info(f"{self.middle_sensor.location} sensor publishing: {msg.range}")
        self.front_ultrasonic_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ultrasonic_publisher = UltrasonicPublisher()
    rclpy.spin(ultrasonic_publisher)
    ultrasonic_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
