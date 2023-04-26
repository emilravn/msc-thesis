import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from .motor_controller import MotorController
from math import atan, pi

# Crop information in lab env
CROP_ROWS = 1
LENGTH_OF_CROP_ROW_CM = 50
WIDTH_OF_ROW_CM = 20

# Robot info
LENGTH_OF_ROBOT_CM = 20

# Goal specifications
DISTANCE_TO_TRAVEL = 40  # cm


class CropFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('crop_follower_node')

        self.desired_dist_to_crop = 30
        self.steering_angle = 0.0
        self.kp = 0.1
        self.speed = 0.5

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

        # Calculate the error
        error = self.desired_dist_to_crop - self.middle_ultrasonic_distance

        # Calculate the sterring angle
        steering_angle = atan(self.kp * error)

        # Set the robot's steering angle

        self.robot.robot.value = (0.5 + steering_angle / (2 * pi), 0.5 - steering_angle / (2 * pi))

        # if self.middle_ultrasonic_distance > self.desired_dist_to_crop:
        #     self.robot.curve_left(0.18)
        #     print(f"curve left with distance value: {self.middle_ultrasonic_distance}")
        # elif self.middle_ultrasonic_distance < self.desired_dist_to_crop - 10:
        #     self.robot.curve_right(0.18)
        #     print(f"curve right with distance value: {self.middle_ultrasonic_distance}")
        # elif self.desired_dist_to_crop - 10 < self.middle_ultrasonic_distance and self.middle_ultrasonic_distance < self.desired_dist_to_crop:
        #     self.robot.drive_forward(0.6)

    def turn_left_around_crop_row(self, enc_dist_cm):
        global CROP_ROWS
        # Drive length of robot to make sure it is clear of the crop row
        if(enc_dist_cm < LENGTH_OF_ROBOT_CM + enc_dist_cm):
            self.robot.motors.forward()
        # Now driven length of robot and clear for turning left around crop row
        self.turn_90_degrees(enc_dist_cm)
        CROP_ROWS = CROP_ROWS - 1

    def turn_90_degrees(self, prev_enc_dist_cm):
        while True:
            current_enc_dist_cm = self.left_encoder_distance / 100
            if current_enc_dist_cm < prev_enc_dist_cm - 240:
                self.robot.motors.left()

    def crop_following_algorithm(self):
        if self.total_encoder is not None and self.back_us is not None:
            while(CROP_ROWS > 0):
                if(self.left_encoder) < LENGTH_OF_CROP_ROW_CM:
                    self.follow_crop()
                else:
                    self.turn_left_around_crop_row(self.left_encoder)


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
