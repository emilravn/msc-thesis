from gpiozero import DistanceSensor


class DistanceSensorImpl:
    def __init__(self, location, gpio_echo, gpio_trigger):
        self.sensor = DistanceSensor(gpio_echo, gpio_trigger, max_distance=4)
        self.location = location

    def get_distance(self):
        """Return distance in centimeters."""
        return self.sensor.distance * 100

    def __str__(self):
        return f"{self.location}"


if __name__ == "__main__":
    try:
        middle_gpio_echo = 27
        middle_gpio_trigger = 17
        us_sensor = DistanceSensorImpl("middle", middle_gpio_echo, middle_gpio_trigger)

        print("Measuring distance...\n")
        while True:
            dist = us_sensor.get_distance()
            print(f"Measured Distance = {dist} cm")
    except KeyboardInterrupt:
        print("Measurement stopped by User")
