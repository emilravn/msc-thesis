#!/usr/bin/python
# -- FROM https://pimylifeup.com/raspberry-pi-distance-sensor/
import RPi.GPIO as GPIO
import time

debug = False


class Sonar():

    def __init__(self, gpio_trigger, gpio_echo):

        GPIO.setmode(GPIO.BCM)

        self._gpio_trigger = gpio_trigger
        self._gpio_echo = gpio_echo

        self._speed_sound = 17150.0  # - divided by 2 in cm/s

        GPIO.setup(gpio_trigger, GPIO.OUT)
        GPIO.setup(gpio_echo, GPIO.IN)

        # - Waiting for sensor to settle
        GPIO.output(gpio_trigger, GPIO.LOW)
        time.sleep(1)

        if debug:
            print("GPIO setup complete")

    def get_distance(self):
        # --- Call for a reading
        GPIO.output(self._gpio_trigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self._gpio_trigger, GPIO.LOW)

        GPIO.output(self._gpio_trigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self._gpio_trigger, GPIO.LOW)

        pulse_start_time = time.time()
        pulse_end_time = time.time()
        # --- Wait for the answer
        while GPIO.input(self._gpio_echo) == 0:
            pulse_start_time = time.time()

        while GPIO.input(self._gpio_echo) == 1:
            pulse_end_time = time.time()

        pulse_duration = pulse_end_time - pulse_start_time
        distance = pulse_duration * self._speed_sound

        return(distance)

    def cleanup(self):
        GPIO.cleanup()


def main():
    GPIO_TRIGGER = 17
    GPIO_ECHO = 27
    us_sensor = Sonar(GPIO_TRIGGER, GPIO_ECHO)

    print("Measuring distance...\n")
    while True:
        dist = us_sensor.get_distance()
        print("Measured Distance = %.1f cm" % dist)


if __name__ == "__main__":
    try:
        main()

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
