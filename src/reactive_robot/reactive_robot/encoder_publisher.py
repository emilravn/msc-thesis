import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from .encoder import Encoders


# Motor encoder setup
L_ENCODER_A = 26
L_ENCODER_B = 16
R_ENCODER_A = 22
R_ENCODER_B = 25

encoders = Encoders(L_ENCODER_A, L_ENCODER_B, R_ENCODER_A, R_ENCODER_B)


class EncoderPublisher(Node):

    def __init__(self):
        super().__init__('encoder_publisher')
        self.left_encoder_publisher_ = self.create_publisher(Float32, "left_encoder/distance", 10)
        self.right_encoder_publisher_ = self.create_publisher(Float32, "right_encoder/distance", 10)
        # self.total_encoder_publisher_ = self.create_publisher(Float32, "total_encoder/distance", 10)
        encoder_timer_period = 0.1
        self.left_encoder_timer = self.create_timer(
            encoder_timer_period, self.left_encoder_callback)
        self.right_encoder_timer = self.create_timer(
            encoder_timer_period, self.right_encoder_callback)
        # self.total_encoder_timer = self.create_timer(
        #     encoder_timer_period, self.total_encoder_callback)

    def left_encoder_callback(self):
        distance_left_encoder = encoders.distance_travelled_left
        msg = Float32()
        msg.data = distance_left_encoder
        self.get_logger().info(f'left encoder publishing: {msg.data}mm')
        self.left_encoder_publisher_.publish(msg)

    def right_encoder_callback(self):
        distance_right_encoder = encoders.distance_travelled_right
        msg = Float32()
        msg.data = distance_right_encoder
        self.get_logger().info(f'right encoder publishing: {msg.data}mm')
        self.right_encoder_publisher_.publish(msg)

    # def total_encoder_callback(self):
    #     distance_covered = encoders.total_distance_travelled()
    #     msg = Float32()
    #     msg.data = distance_covered
    #     self.get_logger().info('total encoder publishing: "%f"' % msg.data)
    #     self.total_encoder_publisher_.publish(msg)


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
