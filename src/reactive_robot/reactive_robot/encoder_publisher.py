import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from .encoder import Encoders


# Motor encoder setup
L_ENCODER_A = 16
L_ENCODER_B = 26
R_ENCODER_A = 22
R_ENCODER_B = 25
encoders = Encoders(L_ENCODER_A, L_ENCODER_B, R_ENCODER_A, R_ENCODER_B)


class EncoderPublisher(Node):

    def __init__(self):
        super().__init__('encoder_publisher')
        self.encoder_publisher_ = self.create_publisher(Float32, "encoder/distance", 10)
        encoder_timer_period = 0.1
        self.encoder_timer = self.create_timer(encoder_timer_period, self.encoder_callback)

    def encoder_callback(self):
        distance_covered = encoders.total_distance_travelled()
        msg = Float32()
        msg.data = distance_covered
        self.get_logger().info('Encoder publishing: "%f"' % msg.data)
        self.encoder_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    encoder_publisher = EncoderPublisher()
    rclpy.spin(encoder_publisher)
    encoders.cleanup_pins()
    encoder_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("encoder_publisher stopped by User")
        encoders.cleanup_pins()
    finally:
        encoders.cleanup_pins()
