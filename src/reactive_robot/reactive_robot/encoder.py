import math
from gpiozero import RotaryEncoder
import RPi.GPIO as GPIO
# from time import sleep


# Encoder/motor specs
GEAR_RATIO = 20.4
BELT_DIAMETER = 87  # millimeter
ENCODER_CPR = 48
COUNTS_PER_REV = ENCODER_CPR * GEAR_RATIO


class Encoder():

    def __init__(self, r_a: int, r_b: int, l_a: int, l_b: int):

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

        self.distance_travelled_left = 0
        self.distance_travelled_right = 0

    def distance_travelled(self, counts):
        """Returns distance in centimeters."""
        revs = counts / COUNTS_PER_REV
        distance = revs * math.pi * BELT_DIAMETER
        return distance

    def on_rotate_left(self):
        global distance_covered_left
        distance_covered_left = self.distance_travelled(self.encoder_left.steps)

    def on_rotate_right(self):
        global distance_covered_right
        distance_covered_right = self.distance_travelled(self.encoder_right.steps)

    def cleanup_pins(self):
        GPIO.cleanup()
