import math
from gpiozero import RotaryEncoder
import RPi.GPIO as GPIO
# from time import sleep


# Encoder/motor specs
GEAR_RATIO = 20.4
BELT_DIAMETER = 180  # millimeter
ENCODER_CPR = 48
COUNTS_PER_REV = ENCODER_CPR * GEAR_RATIO


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

        self.distance_travelled_left = 0.0
        self.distance_travelled_right = 0.0

        self.initial_angle = 0
        self.current_angle = 0

    def distance_travelled(self, counts):
        """Return measured travel distance in millimeters for a single encoder."""
        revs = counts / COUNTS_PER_REV
        distance = revs * math.pi * BELT_DIAMETER

        return distance

    # TODO: check if this is the correct way to combine encoder measurements
    def total_distance_travelled(self):
        """Return the total distance measured by both encoders."""
        return (self.distance_travelled_left + self.distance_travelled_right) / 2

    def on_rotate_left(self):
        self.distance_travelled_left = self.distance_travelled(self.encoder_left.steps)

    def on_rotate_right(self):
        self.distance_travelled_right = self.distance_travelled(self.encoder_right.steps)

    def cleanup_pins(self):
        GPIO.cleanup()
