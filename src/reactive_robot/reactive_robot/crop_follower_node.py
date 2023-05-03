import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from .motor_controller import MotorController
from math import atan, pi
from time import sleep
from enum import Enum

# Crop information in lab env
CROP_ROWS = 2
LENGTH_OF_CROP_ROW = 140  # Plastic box cm
WIDTH_OF_ROW = 33  # Plastic box cm
CLEARANCE = 5  # distance in cm to drive to provide proper clearance from crop

# Algorithm global variables
left_encoder_on_arrival = 0
total_encoder_on_arrival = 0
num_left_turns = 0
crop_rows_done = 0

# Robot info
LENGTH_OF_ROBOT = 28


class State(Enum):
    ANALYZE = 0
    CLEARANCE = 1
    LEFT = 2
    WIDTH = 3
    END = 4


class CropFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('crop_follower_node')

        self.desired_dist_to_crop = 30
        self.steering_angle = 0.0
        self.kp = 0.05  # the best kp
        self.kd = 5  # the best Derivative gain
        self.speed = 0.6
        self.prev_error = 0

        self.robot = MotorController()

        self.state = State()

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

        self.left_encoder_distance = 0.0
        self.right_encoder_distance = 0.0
        self.total_encoder_distance = 0.0

        self.create_timer(0.01, self.crop_following_algorithm)

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

        # steering_angle = max(min(steering_angle, 1.0), -1.0)

        # Set the robot's steering angle
        if self.total_encoder_distance < 1:
            self.robot.set_speed(1, 1)
        else:
            self.robot.set_speed(self.speed + steering_angle / (2 * pi), self.speed - steering_angle / (2 * pi))

        # Update the previous error
        self.prev_error = error

    def crop_following_algorithm(self):
        global total_encoder_on_arrival, num_left_turns, crop_rows_done

        if self.state == State.ANALYZE:
            self.get_logger().info(f'{self.state}')
            self.follow_crop()
            if self.total_encoder_distance >= LENGTH_OF_CROP_ROW:
                self.state = State.CLEARANCE
            elif self.total_encoder_distance >= total_encoder_on_arrival + LENGTH_OF_CROP_ROW:
                self.state = State.END
        elif self.state == State.CLEARANCE:
            self.get_logger().info(f'{self.state}')
            self.robot.drive_forward()
            if (self.total_encoder_distance >= total_encoder_on_arrival +
                    self.desired_dist_to_crop + CLEARANCE) and num_left_turns == 2:
                self.state = State.ANALYZE
                total_encoder_on_arrival = self.total_encoder_distance
            elif self.total_encoder_distance >= (LENGTH_OF_CROP_ROW +
                                                 self.desired_dist_to_crop +
                                                 CLEARANCE):
                self.state = State.LEFT
        elif self.state == State.LEFT:
            self.get_logger().info(f'{self.state}')
            self.robot.set_speed(-1, 1)
            sleep(1.0)
            total_encoder_on_arrival = self.total_encoder_distance
            if num_left_turns < 1:
                self.state = State.WIDTH
            else:
                self.state = State.CLEARANCE
            num_left_turns += 1
        elif self.state == State.WIDTH:
            self.get_logger().info(f'{self.state}')
            self.follow_crop()
            if self.total_encoder_distance >= total_encoder_on_arrival + self.desired_dist_to_crop*2:
                self.state = State.LEFT
        elif self.state == State.END:
            self.get_logger().info(f'{self.state}')
            if crop_rows_done == CROP_ROWS:
                self.state = State.CLEARANCE
                total_encoder_on_arrival = self.total_encoder_distance
            else:
                crop_rows_done += 1
                num_left_turns = 0
                total_encoder_on_arrival = 0
                self.robot.set_speed(-1, 1)
                sleep(1.5)


def main(args=None):
    rclpy.init(args=args)
    crop_follower_node = CropFollowerNode()
    rclpy.spin(crop_follower_node)
    crop_follower_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
