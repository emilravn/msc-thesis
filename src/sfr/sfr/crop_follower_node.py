#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Float32MultiArray
from .motor_driver import MotorDriver
from math import atan, pi
from enum import Enum
from time import sleep
import time
from .camera_driver import capture_plain_image

# Crop information in lab env
CROP_ROWS = 4
CROP_ROW_LENGTH = 180  # cm
WIDTH_OF_ROW = 75  # cm
CLEARANCE = 10  # distance in cm to drive to provide proper clearance from crop
DESIRED_DIST_TO_CROP = 20  # desired distance the robot should be from the crop

# Robot info
LENGTH_OF_ROBOT = 28
WIDTH_OF_ROBOT = 25

# Distances to the algorithm
CROP_ROW_CLEARANCE_DIST = CROP_ROW_LENGTH + CLEARANCE * 2 + WIDTH_OF_ROW
CLEARANCE_DIST = LENGTH_OF_ROBOT + CLEARANCE
SECOND_ANALYZE_DIST = CROP_ROW_CLEARANCE_DIST + CROP_ROW_LENGTH
WIDTH_DIST = WIDTH_OF_ROW + DESIRED_DIST_TO_CROP * 3
SUM_CROP_DISTANCE = SECOND_ANALYZE_DIST + CROP_ROW_LENGTH

# PID controller
MAX_VELOCITY = 1
MAX_STEERING_ANGLE = 1
MAX_INTEGRAL_ERROR = 1

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
        super().__init__("crop_follower_node")

        self.steering_angle = 0.0
        self.kp = 0.2  # the best kp = 0.2
        self.kd = 0.8  # the best Derivative gain = 0.8
        self.ki = 0.01  # the best Integral gain = 0.01
        self.speed = 0.6  # base speed
        self.prev_error = 0.0
        self.integral_error = 0.0  # initialize integral error
        self.last_stop_cm = 0.0  # when we last stopped
        self.should_turn_left = False

        self.robot = MotorDriver()

        self.image_no = 0

        self.state = State.ANALYZE

        self.back_ultrasonic_subscription = self.create_subscription(
            Range, "ultrasonic/back/distance", self.back_ultrasonic_listener_callback, 10
        )

        self.middle_ultrasonic_subscription = self.create_subscription(
            Range, "ultrasonic/middle/distance", self.middle_ultrasonic_listener_callback, 10
        )

        self.front_ultrasonic_subscription = self.create_subscription(
            Range, "ultrasonic/front/distance", self.front_ultrasonic_listener_callback, 10
        )

        self.back_ultrasonic_distance = 0.0
        self.middle_ultrasonic_distance = 0.0
        self.front_ultrasonic_distance = 0.0

        self.right_encoder_subscription = self.create_subscription(
            Float32, "right_encoder/distance", self.right_encoder_listener_callback, 10
        )

        self.left_encoder_subscription = self.create_subscription(
            Float32, "left_encoder/distance", self.left_encoder_listener_callback, 10
        )

        self.total_encoder_subscription = self.create_subscription(
            Float32, "total_encoder/distance", self.total_encoder_listener_callback, 10
        )

        self.left_encoder_distance = 0.0
        self.right_encoder_distance = 0.0
        self.total_encoder_distance = 0.0

        self.scd30_subscription = self.create_subscription(
            Float32MultiArray, "scd30/c02_temperature_humidity", self.scd30_listener_callback, 10
        )

        self.scd30_c02_temp_hum = [0, 0, 0]

        # self.create_timer(0.01, self.crop_following_algorithm)
        self.create_timer(0.01, self.follow_crop)
        # self.create_timer(0.01, self.make_perfect_square_experiment)

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

    def scd30_listener_callback(self, msg: Float32MultiArray):
        self.scd30_c02_temp_hum = msg.data

    def follow_crop(self):
        distances = [
            self.middle_ultrasonic_distance,
            self.front_ultrasonic_distance,
            self.back_ultrasonic_distance,
        ]
        angles = [0, pi / 10, -(pi / 10)]

        # Find the sensor with the lowest distance reading
        min_distance = min(distances)
        min_index = distances.index(min_distance)
        distance_to_crop = min_distance
        steering_angle = angles[min_index]

        # Calculate the error
        error = DESIRED_DIST_TO_CROP - distance_to_crop

        # Calculate the derivative term
        delta_error = error - self.prev_error
        derivative = self.kd * delta_error

        # Calculate the integral term
        self.integral_error += error
        self.integral_error = max(
            min(self.integral_error, MAX_INTEGRAL_ERROR), -MAX_INTEGRAL_ERROR
        )
        integral = self.ki * self.integral_error

        # Calculate the steering angle
        if steering_angle == 0.0:
            steering_angle = 0.00000001

        self.steering_angle = (
            atan(self.kp * error + derivative + integral) * steering_angle / abs(steering_angle)
        )

        kickstart = 2

        if self.total_encoder_distance < kickstart:
            self.robot.set_speed(1, 1)

        # elif self.total_encoder_distance - self.last_stop_cm >= 20:
        #     self.get_logger().info(
        #         "Collecting environmental data: "
        #         + f"CO2 = {self.scd30_c02_temp_hum[0]}, "
        #         + f"temperature = {self.scd30_c02_temp_hum[1]}, "
        #         + f"humidity = {self.scd30_c02_temp_hum[2]}"
        #     )
        #     self.get_logger().info(f"Distance to wall at stop = {min_distance}")
        #     self.get_logger().info(f"Encoder stop = {self.total_encoder_distance}")
        #     self.wait(1, action=self.robot.stop)
        #     self.last_stop_cm = self.total_encoder_distance
        #     self.robot.set_speed(1, 1)
        elif self.total_encoder_distance > 180:  # stop after experiment
            self.robot.stop()
        else:
            left_velocity = max(
                min(self.speed + self.steering_angle / (2 * pi), MAX_VELOCITY), -MAX_VELOCITY
            )
            right_velocity = max(
                min(self.speed - self.steering_angle / (2 * pi), MAX_VELOCITY), -MAX_VELOCITY
            )
            self.robot.set_speed(left_velocity, right_velocity)

        self.prev_error = error

    def wait(self, duration, action=None):
        current_time = time.time()
        while time.time() < current_time + duration:
            if action:
                action()

    def stop_and_take_data_sample(self):
        # self.get_logger().info("Stopping..")
        self.robot.set_speed(0, 0)
        # self.get_logger().info(
        #     f"Collecting environmental data: CO2 = {self.scd30_c02_temp_hum[0]}, "
        #     + f"temperature = {self.scd30_c02_temp_hum[1]}, humidity = {self.scd30_c02_temp_hum[2]}"
        # )
        # self.get_logger().info("Capturing image..")
        # capture_plain_image(self.image_no)
        # self.image_no = self.image_no + 1

    def make_perfect_square_experiment(self):
        global num_left_turns, total_encoder_on_arrival, left_encoder_on_arrival

        distance_for_each_turn = 100

        if self.should_turn_left:
            if self.left_encoder_distance > left_encoder_on_arrival - 10:
                self.robot.turn_left()
            else:
                self.should_turn_left = False
        # left turn
        elif self.total_encoder_distance >= self.last_stop_cm + distance_for_each_turn:
            self.should_turn_left = True
            self.last_stop_cm = self.total_encoder_distance
            left_encoder_on_arrival = self.left_encoder_distance
        elif self.total_encoder_distance > 400:
            self.robot.stop()
        else:  # drive forward
            self.robot.set_speed(0.8, 0.8)

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
        # self.get_logger().info(f"{self.state}")

        # Start of crop row or start of other side of crop row
        if (
            self.total_encoder_distance < CROP_ROW_LENGTH
            or self.total_encoder_distance < SUM_CROP_DISTANCE
            and num_left_turns == 2
        ):
            self.follow_crop()
            if self.total_encoder_distance >= SUM_CROP_DISTANCE - 5 and num_left_turns == 2:
                crop_rows_done += 1
                if crop_rows_done >= CROP_ROWS:
                    self.state = State.HOME
                else:
                    # self.get_logger().info(f"Total distance driven: {self.total_encoder_distance}")
                    left_encoder_on_arrival = self.left_encoder_distance
                    self.state = State.END

        # End of first side crop row
        elif self.total_encoder_distance >= CROP_ROW_LENGTH and num_left_turns < 1:
            total_encoder_on_arrival = self.total_encoder_distance
            crop_rows_done += 1
            self.state = State.CLEARANCE
        # End of second side crop row
        elif self.total_encoder_distance <= SUM_CROP_DISTANCE - 5 and num_left_turns > 1:
            crop_rows_done += 1
            if crop_rows_done >= CROP_ROWS:
                self.state = State.HOME
            else:
                # self.get_logger().info(f"Total distance driven: {self.total_encoder_distance}")
                left_encoder_on_arrival = self.left_encoder_distance
                self.state = State.END

    def state_clearance(self):
        global total_encoder_on_arrival, left_encoder_on_arrival, num_left_turns
        # self.get_logger().info(f"State = {self.state}")

        # First or second clearance
        if self.total_encoder_distance < total_encoder_on_arrival + CLEARANCE_DIST:
            self.robot.set_speed(1, 1)
        # Either going for the first left turn or into second analyze
        elif self.total_encoder_distance >= total_encoder_on_arrival + CLEARANCE_DIST:
            total_encoder_on_arrival = self.total_encoder_distance
            if num_left_turns < 1:
                total_encoder_on_arrival = self.total_encoder_distance
                left_encoder_on_arrival = self.left_encoder_distance
                self.state = State.LEFT
            else:
                self.state = State.ANALYZE

    def state_left(self):
        global total_encoder_on_arrival, left_encoder_on_arrival, num_left_turns
        # self.get_logger().info(f"{self.state}")

        if self.left_encoder_distance > left_encoder_on_arrival - 10:
            self.robot.set_speed(-1, 1)
        else:
            total_encoder_on_arrival = self.total_encoder_distance
            if num_left_turns < 1:
                self.state = State.WIDTH
            else:
                self.state = State.CLEARANCE
            num_left_turns += 1

    def state_width(self):
        global total_encoder_on_arrival, left_encoder_on_arrival
        # self.get_logger().info(f"{self.state}")

        if self.total_encoder_distance < WIDTH_DIST + total_encoder_on_arrival:
            # self.follow_crop()
            self.robot.set_speed(1, 1)
        else:
            left_encoder_on_arrival = self.left_encoder_distance
            self.state = State.LEFT

    def state_end(self):
        global crop_rows_done, num_left_turns, total_encoder_on_arrival, left_encoder_on_arrival
        # self.get_logger().info(f"{self.state}")

        if self.left_encoder_distance > left_encoder_on_arrival - 30:
            self.robot.set_speed(-1, 1)
        else:
            self.reset_global_variables()
            self.state = State.ANALYZE

    def state_home(self):
        # self.get_logger().info("Go home")
        self.robot.set_speed(0, 0)

    def reset_global_variables(self):
        global crop_rows_done, num_left_turns, total_encoder_on_arrival, left_encoder_on_arrival
        num_left_turns = 0
        total_encoder_on_arrival = 0
        left_encoder_on_arrival = 0


def main(args=None):
    rclpy.init(args=args)
    crop_follower_node = CropFollowerNode()
    rclpy.spin(crop_follower_node)
    sleep(1)
    crop_follower_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
