import math
from gpiozero import RotaryEncoder, Robot, PWMOutputDevice, Motor
import RPi.GPIO as GPIO          
from time import sleep

# L298N motor driver pins
# INA and INB corresponds to the right motor, INC and IND to the left motor

# RIGHT
MOTOR_INA = 24
MOTOR_INB = 23
MOTOR_ENA = 12
# LEFT
MOTOR_IND = 6
MOTOR_INC = 5
MOTOR_ENB = 13

# Encoder pins
L_ENCODER_A = 26
L_ENCODER_B = 16
R_ENCODER_A = 25
R_ENCODER_B = 22

DEFAULT_MOTOR_SPEED = 60

robot = Robot(left=(MOTOR_INC, MOTOR_IND, MOTOR_ENA), right=(MOTOR_INA, MOTOR_INB, MOTOR_ENB), pwm=False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_ENA,GPIO.OUT)
GPIO.setup(MOTOR_ENB,GPIO.OUT)
pwm_right = GPIO.PWM(MOTOR_ENA, 20)
pwm_left = GPIO.PWM(MOTOR_ENB, 20)
pwm_right.start(DEFAULT_MOTOR_SPEED)
pwm_left.start(DEFAULT_MOTOR_SPEED)
# right_motor = Motor(MOTOR_INA, MOTOR_INB, pwm=False)
# left_motor = Motor(MOTOR_INC, MOTOR_IND, pwm=False)


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

# Define the distance to travel (in cm)
distance_to_travel = 400
distance_covered_left = 0
distance_covered_right = 0
distance_travelled = 0

# Encoder/motor specs 
gear_ratio = 20.4
wheel_diameter = 48 # millimeter
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

            if abs(total_distance_covered) > distance_to_travel:
                robot.stop()
                break

    except KeyboardInterrupt:
        print("Stopped")
    finally:
        GPIO.cleanup()
