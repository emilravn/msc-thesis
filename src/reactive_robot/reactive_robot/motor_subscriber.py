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

# Goal specifications
DISTANCE_TO_TRAVEL = 40  # cm


class Subscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        self.ultrasonic_subscription = self.create_subscription(
            Range,
            'ultrasonic/distance',
            self.ultrasonic_listener_callback,
            10)
        self.ultrasonic_subscription  # prevent unused variable warning

        self.encoder_subscription = self.create_subscription(
            Float32,
            'encoder/distance',
            self.encoder_listener_callback,
            10)
        self.encoder_subscription

    def ultrasonic_listener_callback(self, msg: Range):
        # self.get_logger().info('Ultrasonic: "%s"' % msg.range)
        # dist_cm = msg.range*100
        dist_cm = 50  # TODO: TEST remove

        if dist_cm > 55:
            # turn left
            robot.motors.left()
        elif (dist_cm < 45):
            # turn right
            robot.motors.right()
        else:
            # drive forward
            robot.motors.forward()

    def encoder_listener_callback(self, msg: Float32):
        self.get_logger().info('Encoder: "%s"' % msg.data)
        distance_travelled = msg.data

        if distance_travelled > 5:
            robot.motors.stop()


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
