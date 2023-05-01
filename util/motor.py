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
                 enA: int, enB: int):

        self.motors = Robot(left=(inC, inD),
                            right=(inA, inB),
                            pwm=False)
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(enA, GPIO.OUT)
        GPIO.setup(enB, GPIO.OUT)
        self.pwm_left = GPIO.PWM(enB, 5)
        self.pwm_right = GPIO.PWM(enA, 5)
        self.pwm_left.start(100)
        self.pwm_right.start(100)

    def set_speed(self, left_speed: float, right_speed: float):
        self.pwm_left.ChangeDutyCycle(left_speed)
        self.pwm_right.ChangeDutyCycle(right_speed)

    def cleanup_pins():
        GPIO.cleanup()


if __name__ == "__main__":
    try:
        robot = Motor(MOTOR_INA, MOTOR_INB, MOTOR_INC, MOTOR_IND,
                      MOTOR_ENA, MOTOR_ENB)
        
        robot.set_speed(40, 40)
        robot.motors.forward()
        sleep(0.3)
        robot.set_speed(100, 100)
        robot.motors.forward()
        sleep(5)

    except KeyboardInterrupt:
        print("Stopped by user")
        GPIO.cleanup()
    finally:
        GPIO.cleanup()
