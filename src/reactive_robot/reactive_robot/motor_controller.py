from gpiozero import Robot
import RPi.GPIO as GPIO
from time import sleep


class MotorController():
    def __init__(self,
                 inA: int = 24, inB: int = 23,
                 inC: int = 5, inD: int = 6,
                 enA: int = 12, enB: int = 13):

        self.motors = Robot(left=(inC, inD), right=(inA, inB), pwm=False)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(enA, GPIO.OUT)
        GPIO.setup(enB, GPIO.OUT)
        self.pwm_left = GPIO.PWM(enB, 20)
        self.pwm_right = GPIO.PWM(enA, 20)
        self.pwm_left.start(100)
        self.pwm_right.start(100)

    def set_speed(self, left_speed: float, right_speed: float):
        self.pwm_left.ChangeDutyCycle(left_speed)
        self.pwm_right.ChangeDutyCycle(right_speed)

    def drive_forward(self):
        self.motors.forward()

    def drive_backward(self):
        self.motors.backward()

    def turn_left(self):
        self.motors.left()

    def turn_right(self):
        self.motors.right()

    def curve_left(self, speed: float):
        self.motors.forward(curve_left=speed)

    def curve_right(self, speed: float):
        self.motors.forward(curve_right=speed)


if __name__ == "__main__":
    robot = MotorController()
    robot.set_speed()
    sleep(10)
