import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from .motor_controller import MotorController
from math import atan, pi
from time import sleep

# Crop information in lab env
CROP_ROWS = 2
LENGTH_OF_CROP_ROW = 90  # Plastic box cm
WIDTH_OF_ROW = 110  # Plastic box cm
CLEARANCE = 5  # distance in cm to drive to provide proper clearance from crop

# Algorithm global variables
left_encoder_on_arrival = 0
total_enc_on_arrival = 0

# Robot info
LENGTH_OF_ROBOT = 28


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

        self.create_timer(0.02, self.crop_following_algorithm)

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

        steering_angle = max(min(steering_angle, 1.0), -1.0)

        # Set the robot's steering angle
        if self.total_encoder_distance < 1 or self.total_encoder_distance > 130 and self.total_encoder_distance < 150:
            self.robot.set_speed(1, 1)
        else:
            self.robot.set_speed(self.speed + steering_angle / (2 * pi), self.speed - steering_angle / (2 * pi))

        # Update the previous error
        self.prev_error = error

    def turn_around_crop_row(self):
        self.get_logger().info('Driving desired dist to crop row')
        self.drive_distance(self.desired_dist_to_crop, self.total_encoder_distance)
        # Now driven dist to crop and clear for turning left around crop row
        self.get_logger().info('Turning left')
        self.turn_90_degrees_left(self.left_encoder_distance)
        # Now drive width of crop
        # self.get_logger().info('Driving width + desired dist to crop row * 2')
        # self.drive_distance(WIDTH_OF_ROW + self.desired_dist_to_crop * 2,
        #                     self.total_encoder_distance)
        # Now turn left around the crop again into lane number 2
        # self.get_logger().info('Turning left again')
        # self.turn_90_degrees_left()
        # Now drive to crop start of new row
        # self.get_logger().info('Driving desired dist to crop row again')
        # self.drive_distance(self.desired_dist_to_crop, self.total_encoder_distance)
        # Reset total distance
        # self.get_logger().info('At end of loop - start all over again')
        # self.total_encoder_distance = 0

    def turn_90_degrees_left(self, enc_dist_on_arrival):
        if self.left_encoder_distance > enc_dist_on_arrival - 24:
            self.robot.set_speed(-1, 1)

    def drive_distance(self, distance_to_drive, enc_dist_on_arrival):
        self.get_logger().info(
            f'total: {self.total_encoder_distance}, d+e: {distance_to_drive + enc_dist_on_arrival}')
        self.robot.drive_forward(self.speed)

    def crop_following_algorithm(self):
        global left_encoder_on_arrival, total_enc_on_arrival

        if self.total_encoder_distance < LENGTH_OF_CROP_ROW:
            self.get_logger().info('Driving length of crop row')
            self.follow_crop()
        elif self.total_encoder_distance < LENGTH_OF_CROP_ROW + CLEARANCE + self.desired_dist_to_crop:
            self.get_logger().info('Driving desired length from crop row')
            self.robot.drive_forward()
            left_encoder_on_arrival = self.left_encoder_distance
        elif self.left_encoder_distance > left_encoder_on_arrival - 13:
            self.get_logger().info(f'Turning left with LE: {self.left_encoder_distance} and LOA: {left_encoder_on_arrival}')
            self.robot.set_speed(-1, 1)
            total_enc_on_arrival = self.total_encoder_distance
        elif self.total_encoder_distance < total_enc_on_arrival + WIDTH_OF_ROW + self.desired_dist_to_crop * 2:
            left_encoder_on_arrival = 1000  # preventing from entering the previous elif check
            self.get_logger().info(f'Driving width + desired distance from crop * 2, t={self.total_encoder_distance}, g={self.total_encoder_distance + WIDTH_OF_ROW + self.desired_dist_to_crop * 2}, l={self.left_encoder_distance}')
            self.follow_crop()
            left_encoder_on_arrival = self.left_encoder_distance
        else:
            self.get_logger().info('STOP')
            self.robot.motors.stop()


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
