#!/usr/bin/python
from gpiozero import DistanceSensor


class Sonar():

    def __init__(self, gpio_echo, gpio_trigger):
        self.sensor = DistanceSensor(gpio_echo, gpio_trigger)

    def get_distance(self):
        return self.sensor.distance * 100


def main():
    # middle
    GPIO_TRIGGER = 17
    GPIO_ECHO = 27

    # front corner
    FRONT_GPIO_TRIGGER = 14
    FRONT_GPIO_ECHO = 15

    # back corner
    BACK_GPIO_TRIGGER = 8
    BACK_GPIO_ECHO = 7
    us_sensor = Sonar(FRONT_GPIO_TRIGGER, FRONT_GPIO_ECHO)

    print("Measuring distance...\n")
    while True:
        dist = us_sensor.get_distance()
        print(f"Measured Distance = {dist} cm")


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print("Measurement stopped by User")
