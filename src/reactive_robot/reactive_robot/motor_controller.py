from gpiozero import Robot, PWMOutputDevice, Motor
import RPi.GPIO as GPIO
from time import sleep


class MotorController():
    def __init__(self,
                 inA: int = 24, inB: int = 23,
                 inC: int = 5, inD: int = 6,
                 enA: int = 12, enB: int = 13):

        self.motors = Robot(left=(inC, inD), right=(inA, inB), pwm=True)

        self.pwmA = PWMOutputDevice(enA, active_high=True, frequency=18, initial_value=0.72)
        self.pwmB = PWMOutputDevice(enB, active_high=True, frequency=18, initial_value=0.72)

    def set_speed(self, left_speed: float, right_speed: float):
        self.motors.value = (left_speed, right_speed)

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
    robot.set_speed(1, 1)
    sleep(0.9)
    robot.set_speed(0.5,0.5)
    sleep(5)

