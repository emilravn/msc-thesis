import math
from gpiozero import RotaryEncoder
import RPi.GPIO as GPIO
from . import motor
# from time import sleep


R_ENCODER_A = 25
R_ENCODER_B = 22
L_ENCODER_A = 26
L_ENCODER_B = 16

encoder_left = RotaryEncoder(
    L_ENCODER_A,
    L_ENCODER_B,
    max_steps=0,
    wrap=False)

encoder_right = RotaryEncoder(
    R_ENCODER_A,
    R_ENCODER_B,
    max_steps=0,
    wrap=False)

# Define the distance to travel (in cm)
distance_to_travel = 40
distance_covered_left = 0
distance_covered_right = 0

# Encoder/motor specs
gear_ratio = 20.4
belt_diameter = 87  # millimeter
encoder_cpr = 48
counts_per_rev = encoder_cpr * gear_ratio


def distance_travelled(counts):
    """Returns distance in centimeters."""
    revs = counts / counts_per_rev
    distance = revs * math.pi * belt_diameter
    return distance


def on_rotate_left():
    global distance_covered_left
    distance_covered_left = distance_travelled(encoder_left.steps)


def on_rotate_right():
    global distance_covered_right
    distance_covered_right = distance_travelled(encoder_right.steps)


def cleanup_pins():
    GPIO.cleanup()


if __name__ == "__main__":
    try:
        motor.pwm_left.ChangeDutyCycle(motor.DEFAULT_MOTOR_SPEED)
        motor.pwm_right.ChangeDutyCycle(motor.DEFAULT_MOTOR_SPEED)
        while True:
            encoder_left.when_rotated = on_rotate_left()
            encoder_right.when_rotated = on_rotate_right()
            motor.robot.forward()
            total_distance_covered = (distance_covered_left + distance_covered_right) / 2
            print(f"total_distance_covered: {abs(total_distance_covered)}")

            if abs(total_distance_covered) > distance_to_travel:
                motor.robot.stop()
                break

    except KeyboardInterrupt:
        print("Stopped")
    finally:
        GPIO.cleanup()
