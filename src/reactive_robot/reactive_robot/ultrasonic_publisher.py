import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from functools import partial

from .ultrasonic import Sonar

middle = Sonar('middle', 27, 17)
back = Sonar('back', 7, 8)
front = Sonar('front', 15, 14)

SPIN_QUEUE = []
PERIOD = 0.01


class UltrasonicPublisher(Node):  # 'MinimalPublisher' is a subclass (inherits) of 'Node'

    def __init__(self, sensor: Sonar, topic):
        super().__init__(sensor.location)
        # 3rd parameter, 'qos_profile' is "queue size"

        # seconds
        timer_period = 0.02
        callback_function = partial(self.callback, sensor)
        self.ultrasonic_publisher_ = self.create_publisher(Range, topic, 10)
        self.ultrasonic_timer = self.create_timer(timer_period, callback_function)

    def callback(self, sensor: Sonar):
        us_distance = sensor.get_distance()
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/sonar_link"
        msg.radiation_type = 0
        msg.field_of_view = 0.0
        msg.min_range = 0.0
        msg.max_range = 10.0
        msg.range = us_distance
        self.get_logger().info(f'{sensor} sensor publishing: "%f"' % msg.range)
        self.ultrasonic_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    back_sensor_node = UltrasonicPublisher(back, 'ultrasonic/back/distance')
    middle_sensor_node = UltrasonicPublisher(middle, 'ultrasonic/middle/distance')
    front_sensor_node = UltrasonicPublisher(front, 'ultrasonic/front/distance')

    SPIN_QUEUE.append(back_sensor_node)
    SPIN_QUEUE.append(middle_sensor_node)
    SPIN_QUEUE.append(front_sensor_node)

    while rclpy.ok():
        try:
            for node in SPIN_QUEUE:
                rclpy.spin_once(node, timeout_sec=(PERIOD / len(SPIN_QUEUE)))
        except Exception as e:
            print(f"something went wrong in the ROS Loop: {e}")

    # Destroy the nodes (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    back_sensor_node.destroy_node()
    middle_sensor_node.destroy_node()
    front_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
