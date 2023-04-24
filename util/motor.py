from gpiozero import Robot
import RPi.GPIO as GPIO
from time import sleep

# L298N motor driver pins
# RIGHT
MOTOR_INA = 24
MOTOR_INB = 23
MOTOR_ENA = 12
# LEFT
MOTOR_IND = 6
MOTOR_INC = 5
MOTOR_ENB = 13


class Motor():

    def __init__(self, inA: int, inB: int, inC: int, inD: int,
                 enA: int, enB: int,
                 initial_speed: float = 60.0):

        self.motors = Robot(left=(inC, inD, enB),
                            right=(inA, inB, enA),
                            pwm=False)
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(enA, GPIO.OUT)
        GPIO.setup(enB, GPIO.OUT)
        self.pwm_left = GPIO.PWM(enB, 20)
        self.pwm_right = GPIO.PWM(enA, 20)
        self.pwm_left.start(100.0)
        self.pwm_right.start(100.0)
        sleep(0.2)  # give motors time to start
        self.set_speed(initial_speed, initial_speed)

    def set_speed(self, left_speed: float, right_speed: float):
        self.pwm_left.ChangeDutyCycle(left_speed)
        self.pwm_right.ChangeDutyCycle(right_speed)

    def cleanup_pins():
        GPIO.cleanup()


if __name__ == "__main__":
    try:
        robot = Motor(MOTOR_INA, MOTOR_INB, MOTOR_INC, MOTOR_IND,
                      MOTOR_ENA, MOTOR_ENB)
        while True:
            robot.motors.forward()
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        GPIO.cleanup()
