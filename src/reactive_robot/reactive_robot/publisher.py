import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

from .ultrasonic import Sonar
from . import encoder

# Ultrasonic sensor setup
GPIO_TRIGGER = 17
GPIO_ECHO = 27
us_sensor = Sonar(GPIO_TRIGGER, GPIO_ECHO)

# Motor encoder setup
encoder.encoder_left.when_rotated = encoder.on_rotate_left()
encoder.encoder_right.when_rotated = encoder.on_rotate_right()


class Publisher(Node):  # 'MinimalPublisher' is a subclass (inherits) of 'Node'

    def __init__(self):
        super().__init__('ultrasonic_publisher')
        # 3rd parameter, 'qos_profile' is "queue size"
        self.ultrasonic_publisher_ = self.create_publisher(Range, 'ultrasonic/distance', 10)
        self.encoder_publisher_ = self.create_publisher(Float32, "encoder/distance", 10)

        ultrasonic_timer_period = 0.1  # seconds
        encoder_timer_period = 0.1

        self.ultrasonic_timer = self.create_timer(ultrasonic_timer_period, self.ultrasonic_callback)
        self.encoder_timer = self.create_timer(encoder_timer_period, self.encoder_callback)

    def ultrasonic_callback(self):
        us_distance = us_sensor.get_distance()*0.01
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/sonar_link"
        msg.radiation_type = 0
        msg.field_of_view = 0.0
        msg.min_range = 0.0
        msg.max_range = 10.0
        msg.range = us_distance
        # self.get_logger().info('Ultrasonic publishing: "%f"' % msg.range)
        self.ultrasonic_publisher_.publish(msg)

    def encoder_callback(self):
        distance_covered = (encoder.distance_covered_left + encoder.distance_covered_right) / 2
        msg = Float32()
        msg.data = distance_covered
        self.get_logger().info('Encoder publishing: "%f"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    us_sensor.cleanup()
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("motor_subscriber stopped by User")
        us_sensor.cleanup()
    finally:
        us_sensor.cleanup()
