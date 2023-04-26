from gpiozero import Robot
from time import sleep


class MotorController():
    def __init__(self, inA: int = 24, inB: int = 23, inC: int = 5, inD: int = 6,
                 enA: int = 12, enB: int = 13):

        self.robot = Robot(left=(inC, inD, enB),
                           right=(inA, inB, enA),
                           pwm=True)

    def set_speed(self, left_speed: float = None, right_speed: float = None):
        if right_speed:
            self.robot.forward(curve_left=right_speed)
        if left_speed:
            self.robot.forward(curve_right=left_speed)
        else:
            self.robot.forward()

    def drive_forward(self, speed: float):
        self.robot.forward(speed)

    def drive_backward(self, speed: float):
        self.robot.backward(speed)

    def turn_left(self):
        self.robot.right()

    def turn_right(self):
        self.robot.left()

    def curve_left(self, speed: float):
        self.robot.forward(curve_left=speed)

    def curve_right(self, speed: float):
        self.robot.forward(curve_right=speed)


if __name__ == "__main__":
    robot = MotorController()
    robot.set_speed()
    sleep(10)
