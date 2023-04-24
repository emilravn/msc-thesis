import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

from .motor import Motor

# L298N motor driver pins
# RIGHT
MOTOR_INA = 24
MOTOR_INB = 23
MOTOR_ENA = 12
# LEFT
MOTOR_IND = 6
MOTOR_INC = 5
MOTOR_ENB = 13

robot = Motor(MOTOR_INA, MOTOR_INB, MOTOR_INC, MOTOR_IND,
              MOTOR_ENA, MOTOR_ENB)

# Crop information in lab env
CROP_ROWS = 1
length_of_crop_row_cm = 50
width_of_row_cm = 20
dist_to_crop_cm = 20

# Robot info
length_of_robot_cm = 20

# Goal specifications
DISTANCE_TO_TRAVEL = 40  # cm


class Subscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        self.ultrasonic_subscription = self.create_subscription(
            Range,
            'ultrasonic/distance',
            self.ultrasonic_listener_callback,
            10)
        self.ultrasonic_subscription  # prevent unused variable warning

        self.encoder_subscription = self.create_subscription(
            Float32,
            'left_encoder/distance',
            self.encoder_listener_callback,
            10)
        self.encoder_subscription

        self.ultrasonic_distance = None
        self.left_encoder_distance = None

        self.timer = self.create_timer(10, self.crop_following_algorithm)

    def ultrasonic_listener_callback(self, msg: Range):
        self.get_logger().info('Ultrasonic: "%s"' % msg.range)
        dist_cm = msg.range
        self.ultrasonic_distance = dist_cm

    def encoder_listener_callback(self, msg: Float32):
        self.get_logger().info('Encoder: "%s"' % msg.data)
        distance_travelled = msg.data
        self.left_encoder_distance = distance_travelled

    def follow_crop(self, us_dist_cm):
        if us_dist_cm > dist_to_crop_cm+1 and us_dist_cm < dist_to_crop_cm-1:
            robot.motors.forward()
        elif us_dist_cm < dist_to_crop_cm-1:
            robot.set_speed(50, 30)
        elif us_dist_cm > dist_to_crop_cm+1:
            robot.set_speed(50, 30)

    def turn_left_around_crop_row(self, enc_dist_cm):
        global CROP_ROWS
        # Drive length of robot to make sure it is clear of the crop row
        if(enc_dist_cm < length_of_robot_cm + enc_dist_cm):
            robot.motors.forward(0.5)
        # Now driven length of robot and clear for turning left around crop row
        self.turn_90_degrees(enc_dist_cm)
        CROP_ROWS = CROP_ROWS - 1

    def turn_90_degrees(self, prev_enc_dist_cm):
        while True:
            current_enc_dist_cm = self.left_encoder_distance / 100
            if current_enc_dist_cm < prev_enc_dist_cm - 240:
                robot.motors.left()

    def crop_following_algorithm(self):
        if self.left_encoder_distance is not None and self.ultrasonic_distance is not None:
            while(CROP_ROWS > 0):
                if(self.left_encoder_distance/100.0) < length_of_crop_row_cm:
                    self.follow_crop(self.left_encoder_distance/100)
                else:
                    self.turn_left_around_crop_row(
                        self.left_encoder_distance/100, self.ultrasonic_distance)


def main(args=None):
    rclpy.init(args=args)

    motor_subscriber = Subscriber()

    rclpy.spin(motor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
