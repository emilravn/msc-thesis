import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from .motor_controller import MotorController
from math import atan, pi
from time import sleep
from enum import Enum

# Crop information in lab env
CROP_ROWS = 4
CROP_ROW_LENGTH = 130  # Plastic box cm
WIDTH_OF_ROW = 78  # Plastic box cm
CLEARANCE = 5  # distance in cm to drive to provide proper clearance from crop
DESIRED_DIST_TO_CROP = 20  # desired distance the robot should be from the crop

# Robot info
LENGTH_OF_ROBOT = 28
WIDTH_OF_ROBOT = 25

# Distances to the algorithm
CROP_ROW_CLEARANCE_DIST = CROP_ROW_LENGTH + DESIRED_DIST_TO_CROP + CLEARANCE*2 + WIDTH_OF_ROW
CLEARANCE_DIST = LENGTH_OF_ROBOT + CLEARANCE
SECOND_ANALYZE_DIST = CROP_ROW_CLEARANCE_DIST + CROP_ROW_LENGTH
WIDTH_DIST = WIDTH_OF_ROW + DESIRED_DIST_TO_CROP*3
SUM_CROP_DISTANCE = SECOND_ANALYZE_DIST + CROP_ROW_LENGTH

# Algorithm global variables
left_encoder_on_arrival = 0
total_encoder_on_arrival = 0
num_left_turns = 0
crop_rows_done = 0


class State(Enum):
    ANALYZE = 0
    CLEARANCE = 1
    LEFT = 2
    WIDTH = 3
    END = 4
    HOME = 5


class CropFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('crop_follower_node')

        self.steering_angle = 0.0
        self.kp = 0.05  # the best kp
        self.kd = 5  # the best Derivative gain
        self.speed = 0.85
        self.prev_error = 0

        self.robot = MotorController()

        self.state = State.ANALYZE

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

        # self.create_timer(0.01, self.crop_following_algorithm)
        self.create_timer(0.01, self.follow_crop)

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
        self.total_encoder_distance = msg.data % SUM_CROP_DISTANCE

    def follow_crop(self):
        global num_left_turns, total_encoder_on_arrival

        distances = [self.middle_ultrasonic_distance,
                     self.front_ultrasonic_distance,
                     self.back_ultrasonic_distance]
        angles = [0, pi/18 - abs(self.steering_angle), -(pi/18 + abs(self.steering_angle))]

        # Find the sensor with the lowest distance reading
        min_distance = min(distances)
        min_index = distances.index(min_distance)
        self.get_logger().info(f'Min distance: {min_distance}')
        distance_to_crop = min_distance
        steering_angle = angles[min_index]

        # Calculate the error
        error = DESIRED_DIST_TO_CROP - distance_to_crop

        # Calculate the derivative term
        delta_error = error - self.prev_error
        derivative = self.kd * delta_error

        # Calculate the steering angle
        if steering_angle == 0:
            steering_angle = 0.001

        steering_angle = atan(self.kp * error + derivative) * steering_angle / abs(steering_angle)

        # steering_angle = max(min(steering_angle, 1.0), -1.0)

        left_velocity = max(min(self.speed + steering_angle / (2 * pi), 1.0), -1.0)
        right_velocity = max(min(self.speed - steering_angle / (2 * pi), 1.0), -1.0)

        self.robot.set_speed(left_velocity, right_velocity)

        # Update the previous error
        self.prev_error = error

    def crop_following_algorithm(self):
        global total_encoder_on_arrival, num_left_turns, crop_rows_done

        match self.state:
            case State.ANALYZE:
                self.state_analyze()
            case State.CLEARANCE:
                self.state_clearance()
            case State.LEFT:
                self.state_left()
            case State.WIDTH:
                self.state_width()
            case State.END:
                self.state_end()
            case State.HOME:
                self.state_home()

    def state_analyze(self):
        global total_encoder_on_arrival, num_left_turns, crop_rows_done, left_encoder_on_arrival
        self.get_logger().info(f'{self.state}')

        # Start of crop row or start of other side of crop row
        if self.total_encoder_distance < CROP_ROW_LENGTH or self.total_encoder_distance < SECOND_ANALYZE_DIST + CROP_ROW_LENGTH and num_left_turns > 1:
            self.follow_crop()
        # End of first side crop row
        elif self.total_encoder_distance >= CROP_ROW_LENGTH and num_left_turns < 1:
            total_encoder_on_arrival = self.total_encoder_distance
            crop_rows_done += 1
            self.state = State.CLEARANCE
        # End of second side crop row
        elif self.total_encoder_distance >= SUM_CROP_DISTANCE - 4:
            crop_rows_done += 1
            if crop_rows_done >= CROP_ROWS:
                self.state = State.HOME
            else:
                left_encoder_on_arrival = self.left_encoder_distance
                self.state = State.END

    def state_clearance(self):
        global total_encoder_on_arrival, left_encoder_on_arrival, num_left_turns
        self.get_logger().info(f'{self.state}')

        # First or second clearance
        if self.total_encoder_distance < total_encoder_on_arrival + CLEARANCE_DIST:
            self.robot.set_speed(1, 1)
        # Either going for the first left turn or into second analyze
        elif self.total_encoder_distance >= total_encoder_on_arrival + CLEARANCE_DIST:
            total_encoder_on_arrival = self.total_encoder_distance
            if num_left_turns < 1:
                left_encoder_on_arrival = self.left_encoder_distance
                self.state = State.LEFT
            else:
                self.state = State.ANALYZE

    def state_left(self):
        global total_encoder_on_arrival, num_left_turns
        self.get_logger().info(f'{self.state}')

        self.robot.set_speed(-1, 1)
        sleep(0.88)
        total_encoder_on_arrival = self.total_encoder_distance
        if num_left_turns < 1:
            self.state = State.WIDTH
        else:
            self.state = State.CLEARANCE
        num_left_turns += 1

    def state_width(self):
        global total_encoder_on_arrival, left_encoder_on_arrival
        self.get_logger().info(f'{self.state}')

        if self.total_encoder_distance < WIDTH_DIST + total_encoder_on_arrival:
            # self.follow_crop()
            self.robot.set_speed(1, 1)
        else:
            left_encoder_on_arrival = self.left_encoder_distance
            self.state = State.LEFT

    def state_end(self):
        global crop_rows_done, num_left_turns, total_encoder_on_arrival, left_encoder_on_arrival
        self.get_logger().info(f'{self.state}')

        self.robot.set_speed(-1, 1)
        sleep(1.66)

        self.state = State.ANALYZE
        self.reset_global_variables()

    def state_home(self):
        self.get_logger().info('Go home')

    def reset_global_variables(self):
        global crop_rows_done, num_left_turns, total_encoder_on_arrival, left_encoder_on_arrival
        num_left_turns = 0
        total_encoder_on_arrival = 0
        left_encoder_on_arrival = 0


def main(args=None):
    rclpy.init(args=args)
    crop_follower_node = CropFollowerNode()
    rclpy.spin(crop_follower_node)
    crop_follower_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
