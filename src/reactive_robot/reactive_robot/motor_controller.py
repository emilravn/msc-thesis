from gpiozero import Robot, PWMOutputDevice
from time import sleep


class MotorController():
    def __init__(self,
                 inA: int = 24, inB: int = 23,
                 inC: int = 5, inD: int = 6,
                 enA: int = 12, enB: int = 13):

        self.motors = Robot(left=(inC, inD), right=(inA, inB), pwm=True)

        self.pwmA = PWMOutputDevice(enA, active_high=True, frequency=30, initial_value=0.75)
        self.pwmB = PWMOutputDevice(enB, active_high=True, frequency=30, initial_value=0.75)

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
    robot.set_speed(0.1, 0.1)
    print("0.1")
    sleep(3)
    robot.set_speed(0.2, 0.2)
    print("0.2")
    sleep(3)
    robot.set_speed(0.3, 0.3)
    print("0.3")
    sleep(3)
    robot.set_speed(0.4, 0.4)
    print("0.4")
    sleep(3)
    robot.set_speed(0.5, 0.5)
    print("0.5")    
    sleep(3)
    robot.set_speed(0.6, 0.6)
    print("0.6")
    sleep(3)
    robot.set_speed(0.7, 0.7)
    print("0.7")
    sleep(3)
    robot.set_speed(0.8, 0.8)
    print("0.8")
    sleep(3)
    robot.set_speed(0.9, 0.9)
    print("0.9")
    sleep(3)
    robot.set_speed(1, 1)
    print("1")
    sleep(3)
