from gpiozero import Robot
import RPi.GPIO as GPIO
# from time import sleep

# L298N motor driver pins
# RIGHT
MOTOR_INA = 24
MOTOR_INB = 23
MOTOR_ENA = 12
# LEFT
MOTOR_IND = 6
MOTOR_INC = 5
MOTOR_ENB = 13

DEFAULT_MOTOR_SPEED = 60


robot = Robot(left=(MOTOR_INC, MOTOR_IND, MOTOR_ENB),
              right=(MOTOR_INA, MOTOR_INB, MOTOR_ENA), pwm=False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_ENA, GPIO.OUT)
GPIO.setup(MOTOR_ENB, GPIO.OUT)
pwm_left = GPIO.PWM(MOTOR_ENB, 20)
pwm_right = GPIO.PWM(MOTOR_ENA, 20)
pwm_left.start(100)
pwm_right.start(100)


def cleanup_pins():
    GPIO.cleanup()
