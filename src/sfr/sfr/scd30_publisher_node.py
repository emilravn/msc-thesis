#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from scd30_i2c import SCD30


class SCD30Publisher(Node):
    def __init__(self):
        super().__init__('scd30_publisher')

        self.scd30 = SCD30()
        self.scd30.set_measurement_interval(2)
        self.scd30.start_periodic_measurement()

        self.scd30_publisher_ = self.create_publisher(
            Float32MultiArray, "scd30/c02_temperature_humidity", 10)

        scd30_timer_period = 2
        self.scd30_timer = self.create_timer(scd30_timer_period, self.scd30_callback)

    def scd30_callback(self):
        msg = Float32MultiArray()
        if self.scd30.get_data_ready():
            m = self.scd30.read_measurement()
            if m is not None:
                msg.data = m
                # self.get_logger().info(
                #     f"SCD30 publishing: CO2: {m[0]:.2f}ppm, " +
                #     f"temp: {m[1]:.2f}'C, humidity: {m[2]:.2f}%")
                self.scd30_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    scd30_publisher = SCD30Publisher()
    rclpy.spin(scd30_publisher)
    scd30_publisher.destroy_node()
    rclpy.shutdown()
