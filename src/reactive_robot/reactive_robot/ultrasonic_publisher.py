import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from .ultrasonic import Sonar

# Ultrasonic sensor setup
GPIO_TRIGGER = 17
GPIO_ECHO = 27
us_sensor = Sonar(GPIO_TRIGGER, GPIO_ECHO)


class UltrasonicPublisher(Node):  # 'MinimalPublisher' is a subclass (inherits) of 'Node'

    def __init__(self):
        super().__init__('ultrasonic_publisher')
        # 3rd parameter, 'qos_profile' is "queue size"

        self.ultrasonic_publisher_ = self.create_publisher(Range, 'ultrasonic/distance', 10)
        ultrasonic_timer_period = 0.2  # seconds
        self.ultrasonic_timer = self.create_timer(
            ultrasonic_timer_period, self.ultrasonic_callback)

    def ultrasonic_callback(self):
        us_distance = us_sensor.get_distance()
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/sonar_link"
        msg.radiation_type = 0
        msg.field_of_view = 0.0
        msg.min_range = 0.0
        msg.max_range = 10.0
        msg.range = us_distance
        self.get_logger().info('Ultrasonic publishing: "%f"' % msg.range)
        self.ultrasonic_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    ultrasonic_publisher = UltrasonicPublisher()

    rclpy.spin(ultrasonic_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    us_sensor.cleanup_pins()
    ultrasonic_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("ultrasonic_publisher stopped by User")
        us_sensor.cleanup_pins()
    finally:
        us_sensor.cleanup_pins()
