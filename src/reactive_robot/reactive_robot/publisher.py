import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from .ultrasonic import Sonar

GPIO_TRIGGER = 17
GPIO_ECHO = 27
us_sensor = Sonar(GPIO_TRIGGER, GPIO_ECHO)

class Publisher(Node): # 'MinimalPublisher' is a subclass (inherits) of 'Node'

    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.publisher_ = self.create_publisher(Range, 'reactive_robot/distance', 10) # 3rd parameter, 'qos_profile' is "queue size"
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        us_distance = us_sensor.get_distance()*0.01
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/sonar_link"
        msg.radiation_type = 0
        msg.field_of_view = 0.0
        msg.min_range = 0.0
        msg.max_range = 10.0
        msg.range = us_distance
        self.get_logger().info('Publishing: "%f"' % msg.range) # get_logger().info publishes msg to console
        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    
    publisher = Publisher() # this is a node

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print("\n\nCLEANING UP\n\n") # TODO: remove
    us_sensor.cleanup()
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        us_sensor.cleanup()
    finally:
        print("FINALLY")
        us_sensor.cleanup()
