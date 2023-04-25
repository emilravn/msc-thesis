#!/usr/bin/python
from gpiozero import DistanceSensor


class Sonar():
    def __init__(self, location, gpio_echo, gpio_trigger):
        self.sensor = DistanceSensor(gpio_echo, gpio_trigger)
        self.location = location

    def get_distance(self):
        "Return distance in centimeters."
        return self.sensor.distance * 100

    def __str__(self):
        return f'{self.location}'


if __name__ == "__main__":
    try:
        GPIO_TRIGGER = 17
        GPIO_ECHO = 27
        us_sensor = Sonar(GPIO_TRIGGER, GPIO_ECHO)

        print("Measuring distance...\n")
        while True:
            dist = us_sensor.get_distance()
            print(f"Measured Distance = {dist} cm")
    except KeyboardInterrupt:
        print("Measurement stopped by User")
