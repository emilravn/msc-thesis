import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from .motor_controller import MotorController
from math import atan, pi

# Crop information in lab env
CROP_ROWS = 2
LENGTH_OF_CROP_ROW_CM = 43  # Plastic box cm
WIDTH_OF_ROW_CM = 33  # Plastic box cm

# Robot info
LENGTH_OF_ROBOT_CM = 28

# Goal specifications
DISTANCE_TO_TRAVEL = 40  # mm


class CropFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('crop_follower_node')

        self.desired_dist_to_crop = 30
        self.steering_angle = 0.0
        self.kp = 0.05  # the best kp
        self.kd = 5  # the best Derivative gain
        self.speed = 0.5
        self.prev_error = 0

        self.robot = MotorController()

        self.back_ultrasonic_subscription = self.create_subscription(
            Range,
            'ultrasonic/back/distance',
            self.back_ultrasonic_listener_callback,
            10)

        self.middle_ultrasonic_subscription = self.create_subscription(
            Range,
            'ultrasonic/middle/distance',
            self.middle_ultrasonic_listener_callback,
            10)

        self.front_ultrasonic_subscription = self.create_subscription(
            Range,
            'ultrasonic/front/distance',
            self.front_ultrasonic_listener_callback,
            10)

        self.back_ultrasonic_distance = 0.0
        self.middle_ultrasonic_distance = 0.0
        self.front_ultrasonic_distance = 0.0

        self.right_encoder_subscription = self.create_subscription(
            Float32,
            'right_encoder/distance',
            self.right_encoder_listener_callback,
            10)

        self.left_encoder_subscription = self.create_subscription(
            Float32,
            'left_encoder/distance',
            self.left_encoder_listener_callback,
            10)

        self.total_encoder_subscription = self.create_subscription(
            Float32,
            'total_encoder/distance',
            self.total_encoder_listener_callback,
            10)

        self.left_encoder_distance = None
        self.right_encoder_distance = None
        self.total_encoder_distance = None

        self.create_timer(0.02, self.follow_crop)

    def back_ultrasonic_listener_callback(self, msg: Range):
        self.back_ultrasonic_distance = msg.range

    def middle_ultrasonic_listener_callback(self, msg: Range):
        self.middle_ultrasonic_distance = msg.range

    def front_ultrasonic_listener_callback(self, msg: Range):
        self.front_ultrasonic_distance = msg.range

    def right_encoder_listener_callback(self, msg: Float32):
        self.right_encoder_distance = msg.data

    def left_encoder_listener_callback(self, msg: Float32):
        self.left_encoder_distance = msg.data

    def total_encoder_listener_callback(self, msg: Float32):
        self.total_encoder_distance = msg.data

    def follow_crop(self):
        distances = [self.middle_ultrasonic_distance,
                     self.front_ultrasonic_distance,
                     self.back_ultrasonic_distance]
        angles = [0, pi/18 - abs(self.steering_angle), -(pi/18 + abs(self.steering_angle))]

        # Find the sensor with the lowest distance reading
        min_distance = min(distances)
        min_index = distances.index(min_distance)
        distance_to_crop = min_distance
        steering_angle = angles[min_index]

        # Calculate the error
        error = self.desired_dist_to_crop - distance_to_crop

        # Calculate the derivative term
        delta_error = error - self.prev_error
        derivative = self.kd * delta_error

        # Calculate the steering angle
        if steering_angle == 0:
            steering_angle = 0.0001

        steering_angle = atan(self.kp * error + derivative) * steering_angle / abs(steering_angle)

        # Set the robot's steering angle
        self.robot.motors.value = (0.5 + steering_angle / (2 * pi), 0.5 - steering_angle / (2 * pi))

        # Update the previous error
        self.prev_error = error

    def turn_left_around_crop_row(self, enc_dist_cm):
        global CROP_ROWS
        # Drive length of robot to make sure it is clear of the crop row
        if enc_dist_cm < LENGTH_OF_ROBOT_CM + enc_dist_cm:
            self.get_logger().info('Driving length of robot to be clear of the crops')
            self.follow_crop()
        # Now driven length of robot and clear for turning left around crop row
        else:
            self.turn_90_degrees(enc_dist_cm)
        CROP_ROWS = CROP_ROWS - 1

    def turn_90_degrees(self, prev_enc_dist_cm):
        current_enc_dist_cm = self.total_encoder_distance / 10
        if current_enc_dist_cm < prev_enc_dist_cm - 240:
            self.get_logger().info('Turning left around crop row now!')
            self.robot.motors.left()

    def crop_following_algorithm(self):
        if self.total_encoder_distance is not None and CROP_ROWS > 0:
            if self.total_encoder_distance / 10 < LENGTH_OF_CROP_ROW_CM:
                self.follow_crop()
            else:
                self.get_logger().info('End of crop')
                self.turn_left_around_crop_row(self.total_encoder_distance)
        else:
            self.robot.motors.stop()
            self.get_logger().info('Done!')


def main(args=None):
    rclpy.init(args=args)
    crop_follower_node = CropFollowerNode()
    rclpy.spin(crop_follower_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    crop_follower_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
