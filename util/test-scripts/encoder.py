import RPi.GPIO as GPIO          
from time import sleep
import math


# Motor and encoder specifications
gear_ratio = 20.4
wheel_diameter = 48 # mm
encoder_cpr = 48
counts_per_rev = encoder_cpr * gear_ratio

# L298N motor driver pins
# INA and INB corresponds to the right motor, INC and IND to the left motor
MOTOR_INA = 24
MOTOR_INB = 23

MOTOR_IND = 6
MOTOR_INC = 5

MOTOR_ENA = 12
MOTOR_ENB = 13
# Encoder pins
L_ENCODER_A = 26
L_ENCODER_B = 16
R_ENCODER_A = 25
R_ENCODER_B = 22

DEFAULT_MOTOR_SPEED = 50

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_INA,GPIO.OUT)
GPIO.setup(MOTOR_INB,GPIO.OUT)
GPIO.setup(MOTOR_ENA,GPIO.OUT)
GPIO.output(MOTOR_INA,GPIO.LOW)
GPIO.output(MOTOR_INB,GPIO.LOW)

GPIO.setup(MOTOR_IND,GPIO.OUT)
GPIO.setup(MOTOR_INC,GPIO.OUT)
GPIO.setup(MOTOR_ENB,GPIO.OUT)
GPIO.output(MOTOR_INC,GPIO.LOW)
GPIO.output(MOTOR_IND,GPIO.LOW)

GPIO.setup(L_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(L_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(R_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(R_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def event_setup():
    # Enable interrupt detection on encoder input pins
    GPIO.add_event_detect(L_ENCODER_A, GPIO.BOTH, callback=encoder_callback_left, bouncetime=500)
    GPIO.add_event_detect(L_ENCODER_B, GPIO.BOTH, callback=encoder_callback_left, bouncetime=500)
    GPIO.add_event_detect(R_ENCODER_A, GPIO.BOTH, callback=encoder_callback_right, bouncetime=500)
    GPIO.add_event_detect(R_ENCODER_B, GPIO.BOTH, callback=encoder_callback_right, bouncetime=500) # TODO: research bouncetime

pA=GPIO.PWM(MOTOR_ENA,20)
pB=GPIO.PWM(MOTOR_ENB,20)
pA.start(100)
pB.start(100)

# Set initial encoder counts
left_count = 0
right_count = 0
left_distance = 0
right_distance = 0


# Define interrupt handlers for encoder signals
def encoder_callback_left(channel):
    global left_count, left_distance
    sleep(0.1)
    GPIO.remove_event_detect(L_ENCODER_A)
    GPIO.remove_event_detect(L_ENCODER_B)

    if GPIO.input(L_ENCODER_A) == GPIO.input(L_ENCODER_B):
        left_count += 1
        print("Incremented left_count") # TODO: remove
    else:
        left_count -= 1
        print("decremented left_count") # TODO: remove
    left_distance = distance_traveled(left_count)

    print("hello big boy")
    sleep(0.1)
    GPIO.add_event_detect(L_ENCODER_A, GPIO.BOTH, callback=encoder_callback_left, bouncetime=500)
    GPIO.add_event_detect(L_ENCODER_B, GPIO.BOTH, callback=encoder_callback_left, bouncetime=500)


def encoder_callback_right(channel):
    global right_count, right_distance
    sleep(0.1)
    GPIO.remove_event_detect(R_ENCODER_A)
    GPIO.remove_event_detect(R_ENCODER_B)

    if GPIO.input(R_ENCODER_A) == GPIO.input(R_ENCODER_B):
        right_count += 1
        print("Incremented right_count") # TODO: remove
    else:
        right_count -= 1
        print("decremented right_count") # TODO: remove
    right_distance = distance_traveled(right_count)

    print("hello big girl")
    sleep(0.1)
    event_setup()


def forward():
    GPIO.output(MOTOR_INA,GPIO.LOW)
    GPIO.output(MOTOR_INB,GPIO.HIGH)

    GPIO.output(MOTOR_IND,GPIO.LOW)
    GPIO.output(MOTOR_INC,GPIO.HIGH)
    pA.ChangeDutyCycle(DEFAULT_MOTOR_SPEED)
    pB.ChangeDutyCycle(DEFAULT_MOTOR_SPEED)

def backward():
    GPIO.output(MOTOR_INA,GPIO.HIGH)
    GPIO.output(MOTOR_INB,GPIO.LOW)

    GPIO.output(MOTOR_IND,GPIO.HIGH)
    GPIO.output(MOTOR_INC,GPIO.LOW)
    pA.ChangeDutyCycle(DEFAULT_MOTOR_SPEED)
    pB.ChangeDutyCycle(DEFAULT_MOTOR_SPEED)

def turn_left():
    pA.ChangeDutyCycle(DEFAULT_MOTOR_SPEED)
    pB.ChangeDutyCycle(DEFAULT_MOTOR_SPEED-10)

def turn_right():
    pA.ChangeDutyCycle(DEFAULT_MOTOR_SPEED-10)
    pB.ChangeDutyCycle(DEFAULT_MOTOR_SPEED)

def stop():
    pA.stop()
    pB.stop()

def full_stop():
    GPIO.output(MOTOR_INA,GPIO.LOW)
    GPIO.output(MOTOR_INB,GPIO.LOW)
    GPIO.output(MOTOR_IND,GPIO.LOW)
    GPIO.output(MOTOR_INC,GPIO.LOW)
    pA.stop()
    pB.stop()
    GPIO.cleanup()


def distance_traveled(counts):
    '''Returns distance in millimeter.'''
    # Convert encoder counts to motor shaft revolutions
    revs = counts / counts_per_rev
    
    # Calculate distance traveled by wheel
    distance = revs * math.pi * wheel_diameter
    
    return distance


def main(): 
    global left_count, left_distance, right_count, right_distance

    while True:
        left_dist_cm = left_distance*0.1
        right_dist_cm = right_distance*0.1
        total_distance_cm = (left_dist_cm+right_dist_cm)/2
        event_setup() 
        forward()
        # print(f"Left motor count = {left_count}, distance = {left_dist_cm}")
        # print(f"Right motor count = {right_count}, distance = {right_dist_cm}")
        # print(f"Total distance covered = {total_distance_cm}")

        if total_distance_cm == 50:
            full_stop()
            print(f"Full stop, total distance = {total_distance_cm}")
            break

if __name__ == "__main__":
    try:
        main()
 
    except KeyboardInterrupt:
        print("Measurement stopped by User")
    finally:
        GPIO.cleanup()