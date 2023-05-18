from math import pi
from gpiozero import RotaryEncoder
import RPi.GPIO as GPIO

# Encoder/motor specs
GEAR_RATIO = 20.4
BELT_DIAMETER_IN_MILLIMETER = 175
ENCODER_COUNTS_PER_REVOLUTION = 48
COUNTS_PER_REV = ENCODER_COUNTS_PER_REVOLUTION * GEAR_RATIO


class Encoders:
    def __init__(self, l_a: int, l_b: int, r_a: int, r_b: int):
        self.left_encoder = RotaryEncoder(l_a, l_b, max_steps=0, wrap=False)

        self.right_encoder = RotaryEncoder(r_a, r_b, max_steps=0, wrap=False)

        self.left_encoder.when_rotated = self.on_rotate_left
        self.right_encoder.when_rotated = self.on_rotate_right

        self._distance_travelled_left = 0.0
        self._distance_travelled_right = 0.0

    def calc_distance_travelled_cm(self, counts):
        """Return measured travel distance in centimeters for a single encoder."""
        revolutions = counts / COUNTS_PER_REV
        distance = revolutions * pi * BELT_DIAMETER_IN_MILLIMETER

        return distance / 10

    def on_rotate_left(self):
        self._distance_travelled_left = self.calc_distance_travelled_cm(self.left_encoder.steps)

    def on_rotate_right(self):
        self._distance_travelled_right = self.calc_distance_travelled_cm(self.right_encoder.steps)

    def get_total_distance_travelled(self):
        """Return the total distance measured by both encoders."""
        return (self._distance_travelled_left + self._distance_travelled_right) / 2

    def get_distance_travelled_left(self):
        return self._distance_travelled_left

    def get_distance_travelled_right(self):
        return self._distance_travelled_right

    def clean_gpio(self):
        GPIO.cleanup()
