import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from . import motor

DEBUG = True

class Subscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(
            Range,
            'reactive_robot/distance',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Range):
        self.get_logger().info('I heard: "%s"' % msg.range)


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
