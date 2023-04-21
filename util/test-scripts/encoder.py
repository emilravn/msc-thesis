import math
from gpiozero import RotaryEncoder
import RPi.GPIO as GPIO
import motor
# from time import sleep

# GPIO pins
L_ENCODER_A = 26
L_ENCODER_B = 16
R_ENCODER_A = 25
R_ENCODER_B = 22

# Encoder/motor specs
GEAR_RATIO = 20.4
BELT_DIAMETER = 87  # millimeter
ENCODER_CPR = 48
COUNTS_PER_REV = ENCODER_CPR * GEAR_RATIO

# Test parameters
DISTANCE_TO_TRAVEL = 40  # cm


class Encoders():

    def __init__(self, l_a: int, l_b: int, r_a: int, r_b: int):

        self.encoder_left = RotaryEncoder(
            l_a,
            l_b,
            max_steps=0,
            wrap=False)

        self.encoder_right = RotaryEncoder(
            r_a,
            r_b,
            max_steps=0,
            wrap=False)

        self.encoder_left.when_rotated = self.on_rotate_left
        self.encoder_right.when_rotated = self.on_rotate_right

        self.distance_travelled_left = 0
        self.distance_travelled_right = 0

    def distance_travelled(self, counts):
        """Returns measured travel distance in centimeters for a single encoder."""
        revs = counts / COUNTS_PER_REV
        distance = revs * math.pi * BELT_DIAMETER
        return distance

    # TODO: check if this is the correct way to combine encoder measurements
    def total_distance_travelled(self):
        """Returns the total distance measured by both encoders."""
        return (self.distance_travelled_left + self.distance_travelled_right) / 2

    def on_rotate_left(self):
        self.distance_travelled_left = self.distance_travelled(self.encoder_left.steps)

    def on_rotate_right(self):
        self.distance_travelled_right = self.distance_travelled(self.encoder_right.steps)

    def cleanup_pins(self):
        GPIO.cleanup()


if __name__ == "__main__":
    try:
        encoders = Encoders(L_ENCODER_A, L_ENCODER_B, R_ENCODER_A, R_ENCODER_B)

        motor.pwm_left.ChangeDutyCycle(motor.DEFAULT_MOTOR_SPEED)
        motor.pwm_right.ChangeDutyCycle(motor.DEFAULT_MOTOR_SPEED)
        while True:
            motor.robot.forward()
            total_distance_covered = encoders.total_distance_travelled()
            print(f"total_distance_covered: {abs(total_distance_covered)}")

            if abs(total_distance_covered) > DISTANCE_TO_TRAVEL:
                motor.robot.stop()
                break

    except KeyboardInterrupt:
        print("Stopped")
    finally:
        GPIO.cleanup()
