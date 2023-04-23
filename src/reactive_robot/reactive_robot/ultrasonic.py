#!/usr/bin/python
from gpiozero import DistanceSensor


class Sonar():

    def __init__(self, gpio_trigger, gpio_echo):
        self.sensor = DistanceSensor(gpio_echo, gpio_trigger)

    def get_distance(self):
        return self.sensor.distance * 100


def main():
    GPIO_TRIGGER = 17
    GPIO_ECHO = 27
    us_sensor = Sonar(GPIO_TRIGGER, GPIO_ECHO)

    print("Measuring distance...\n")
    while True:
        dist = us_sensor.get_distance()
        print(f"Measured Distance = {dist} cm")


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print("Measurement stopped by User")
