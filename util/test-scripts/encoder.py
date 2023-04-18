import math
from gpiozero import RotaryEncoder, Robot
import RPi.GPIO as GPIO          
from time import sleep

# L298N motor driver pins
# INA and INB corresponds to the right motor, INC and IND to the left motor

# RIGHT
MOTOR_INA = 24
MOTOR_INB = 23
# LEFT
MOTOR_IND = 6
MOTOR_INC = 5

MOTOR_ENA = 12
MOTOR_ENB = 13
# Encoder pins
L_ENCODER_A = 26
L_ENCODER_B = 16
R_ENCODER_A = 25
R_ENCODER_B = 22

robot = Robot(left=(MOTOR_INC, MOTOR_IND, MOTOR_ENA), right=(MOTOR_INA, MOTOR_INB, MOTOR_ENB))

# Create an instance of the RotaryEncoder class
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

DEFAULT_MOTOR_SPEED = 30

# Define the distance to travel (in cm)
distance_to_travel = 50
distance_covered_left = 0
distance_covered_right = 0
distance_travelled = 0

# Encoder/motor specs
gear_ratio = 20.4
wheel_diameter = 48
encoder_cpr = 48
counts_per_rev = encoder_cpr * gear_ratio

def distance_travelled(counts):
    revs = counts / counts_per_rev
    distance = revs * math.pi * wheel_diameter
    return distance

# Define a callback function to be called each time the encoder position changes
def on_rotate_left():
    global distance_covered_left
    distance_covered_left = distance_travelled(encoder_left.steps)

def on_rotate_right():
    global distance_covered_right
    distance_covered_right = distance_travelled(encoder_right.steps)

if __name__ == "__main__":
    try:
        while True:
            # Attach the callback function to the RotaryEncoder instance
            encoder_left.when_rotated = on_rotate_left()
            encoder_right.when_rotated = on_rotate_right()
            robot.forward()
            total_distance_covered = (distance_covered_left + distance_covered_right) / 2
            print(f"total_distance_covered: {abs(total_distance_covered)}")

            if abs(total_distance_covered) >= distance_to_travel:
                robot.stop()

    except KeyboardInterrupt:
        print("Stopped")
    finally:
        GPIO.cleanup()
