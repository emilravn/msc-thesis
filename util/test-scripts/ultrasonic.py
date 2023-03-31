#!/usr/bin/python
#-- FROM https://pimylifeup.com/raspberry-pi-distance-sensor/
import RPi.GPIO as GPIO
import time

debug = False 

class Sonar():

    def __init__(self, gpio_trigger, gpio_echo, range_min=10, range_max=400):
    
        GPIO.setmode(GPIO.BCM)
        
        self._gpio_trigger  = gpio_trigger
        self._gpio_echo     = gpio_echo
        self._range_min     = range_min
        self._range_max     = range_max
        self._is_reading    = False
        
        self._speed_sound   = 17150.0 #- divided by 2 in cm/s
        
        self._last_time_reading = 0
        self._timeout       = range_max/self._speed_sound*2

        GPIO.setup(gpio_trigger, GPIO.OUT)
        GPIO.setup(gpio_echo, GPIO.IN)

        #- Waiting for sensor to settle
        GPIO.output(gpio_trigger, GPIO.LOW)
        time.sleep(1)

        if debug: print("Setup complete")
        

    def get_distance(self):
        self._is_reading = True
        #--- Call for a reading
        GPIO.output(self._gpio_trigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self._gpio_trigger, GPIO.LOW)
        
        GPIO.output(self._gpio_trigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self._gpio_trigger, GPIO.LOW)
        
        pulse_start_time = time.time()
        pulse_end_time = time.time()
        #--- Wait for the answer
        while GPIO.input(self._gpio_echo)==0:
            pulse_start_time = time.time()
            
        time0= time.time()
        while GPIO.input(self._gpio_echo)==1:
            pulse_end_time = time.time()
            # if time.time() - time0 > self._timeout:
                # self._is_reading = False
                # # print time.time() - time0
                # # print self._timeout
                # # print("TIMEOUT")
                # return (-1)
            
        self._last_time_reading = time.time()
        self._is_reading = False

        pulse_duration = pulse_end_time - pulse_start_time
        distance = pulse_duration * self._speed_sound
        
        if distance > self._range_max:
            distance = self._range_max
            
        if distance < self._range_min:
            distance = self._range_min
            # distance = -1
            
        return(distance)

    @property
    def is_reading(self):
        return(self._is_reading)
        
if __name__ == "__main__":
    try:
        GPIO_TRIGGER = 17
        GPIO_ECHO = 27
        us_sensor = Sonar(GPIO_TRIGGER, GPIO_ECHO, 0, 400)

        print("Measuring distance...\n")
        while True:
            dist = us_sensor.get_distance()
            print ("Measured Distance = %.1f cm" % dist)
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()