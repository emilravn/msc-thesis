import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

from .motor import Motor

# L298N motor driver pins
# RIGHT
MOTOR_INA = 24
MOTOR_INB = 23
MOTOR_ENA = 12
# LEFT
MOTOR_IND = 6
MOTOR_INC = 5
MOTOR_ENB = 13

robot = Motor(MOTOR_INA, MOTOR_INB, MOTOR_INC, MOTOR_IND,
              MOTOR_ENA, MOTOR_ENB)

# Crop information in lab env
CROP_ROWS = 1
length_of_crop_row_cm = 50
width_of_row_cm = 20

# Robot info
length_of_robot_cm = 20

# Goal specifications
DISTANCE_TO_TRAVEL = 40  # cm


class Subscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')  # type: ignore

        self.back_ultrasonic_subscription = self.create_subscription(
            Range,
            'ultrasonic/back/distance',
            self.back_ultrasonic_listener_callback,
            10)
        self.back_ultrasonic_subscription

        self.middle_ultrasonic_subscription = self.create_subscription(
            Range,
            'ultrasonic/middle/distance',
            self.middle_ultrasonic_listener_callback,
            10)
        self.middle_ultrasonic_subscription

        self.front_ultrasonic_subscription = self.create_subscription(
            Range,
            'ultrasonic/front/distance',
            self.front_ultrasonic_listener_callback,
            10)
        self.front_ultrasonic_subscription

        self.back_ultrasonic_distance = None
        self.middle_ultrasonic_distance = None
        self.front_ultrasonic_distance = None

        self.encoder_subscription = self.create_subscription(
            Float32,
            'left_encoder/distance',
            self.encoder_listener_callback,
            10)
        self.encoder_subscription

    def back_ultrasonic_listener_callback(self, msg: Range):
        self.get_logger().info(f'back distance: {msg.range}')
        dist_cm = msg.range
        self.back_ultrasonic_distance = dist_cm

    def middle_ultrasonic_listener_callback(self, msg: Range):
        self.get_logger().info(f'middle distance: {msg.range}')
        dist_cm = msg.range
        self.middle_ultrasonic_distance = dist_cm

    def front_ultrasonic_listener_callback(self, msg: Range):
        self.get_logger().info(f'front distance: {msg.range}')
        dist_cm = msg.range
        self.front_ultrasonic_distance = dist_cm

    def encoder_listener_callback(self, msg: Float32):
        self.get_logger().info('Encoder: "%s"' % msg.data)
        distance_travelled = msg.data
        self.left_encoder_distance = distance_travelled


def main(args=None):
    rclpy.init(args=args)

    motor_subscriber = Subscriber()

    rclpy.spin(motor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
