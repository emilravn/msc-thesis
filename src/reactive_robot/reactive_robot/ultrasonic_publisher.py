import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from .ultrasonic import Sonar


class UltrasonicPublisher(Node):  # 'MinimalPublisher' is a subclass (inherits) of 'Node'

    def __init__(self):
        super().__init__('ultrasonic_publisher')

        self.back_sensor = Sonar('back', 7, 8)
        self.middle_sensor = Sonar('middle', 27, 17)
        self.front_sensor = Sonar('front', 15, 14)

        # publishers
        self.back_ultrasonic_publisher_ = self.create_publisher(
            Range, 'ultrasonic/back/distance', 10)
        self.middle_ultrasonic_publisher_ = self.create_publisher(
            Range, 'ultrasonic/middle/distance', 10)
        self.front_ultrasonic_publisher_ = self.create_publisher(
            Range, 'ultrasonic/front/distance', 10)

        # seconds
        timer_period = 0.01

        # timers
        self.back_ultrasonic_publisher_timer = self.create_timer(
            timer_period, self.back_sensor_callback)
        self.middle_ultrasonic_publisher_timer = self.create_timer(
            timer_period, self.middle_sensor_callback)
        self.front_ultrasonic_publisher_timer = self.create_timer(
            timer_period, self.front_sensor_callback)

    def back_sensor_callback(self):
        us_distance = self.back_sensor.get_distance()
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/sonar_link"
        msg.radiation_type = 0
        msg.field_of_view = 0.0
        msg.min_range = 0.0
        msg.max_range = 100.0
        msg.range = us_distance
        self.get_logger().info(f'{self.back_sensor.location} publishing: {msg.range}')
        self.back_ultrasonic_publisher_.publish(msg)

    def middle_sensor_callback(self):
        us_distance = self.middle_sensor.get_distance()
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/sonar_link"
        msg.radiation_type = 0
        msg.field_of_view = 0.0
        msg.min_range = 0.0
        msg.max_range = 100.0
        msg.range = us_distance
        self.get_logger().info(f'{self.middle_sensor.location} sensor publishing: {msg.range}')
        self.middle_ultrasonic_publisher_.publish(msg)

    def front_sensor_callback(self):
        us_distance = self.front_sensor.get_distance()
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/sonar_link"
        msg.radiation_type = 0
        msg.field_of_view = 0.0
        msg.min_range = 0.0
        msg.max_range = 100.0
        msg.range = us_distance
        self.get_logger().info(f'{self.front_sensor.location}sensor publishing: {msg.range}')
        self.front_ultrasonic_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ultrasonic_publisher = UltrasonicPublisher()
    rclpy.spin(ultrasonic_publisher)
    ultrasonic_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
