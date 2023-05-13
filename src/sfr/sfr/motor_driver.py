from gpiozero import Robot, PWMOutputDevice


class MotorDriver:
    def __init__(
        self,
        inA: int = 24,
        inB: int = 23,
        inC: int = 5,
        inD: int = 6,
        enA: int = 12,
        enB: int = 13,
    ):
        self.motors = Robot(left=(inC, inD), right=(inA, inB), pwm=True)
        self.pwmA = PWMOutputDevice(enA, active_high=True, frequency=30, initial_value=0.75)
        self.pwmB = PWMOutputDevice(enB, active_high=True, frequency=30, initial_value=0.75)

    def set_speed(self, left_speed: float, right_speed: float):
        self.motors.value = (left_speed, right_speed)

    def get_speed(self):
        return self.motors.value

    def stop(self):
        self.motors.stop()

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
