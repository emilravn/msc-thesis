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
R_ENCODER_A = 9
R_ENCODER_B = 11

DEFAULT_MOTOR_SPEED = 30

GPIO.setmode(GPIO.BCM)

GPIO.setup(MOTOR_INA,GPIO.OUT)
GPIO.setup(MOTOR_INB,GPIO.OUT)
GPIO.setup(MOTOR_ENA,GPIO.OUT)
GPIO.output(MOTOR_INA,GPIO.LOW)
GPIO.output(MOTOR_INB,GPIO.LOW)

GPIO.setup(MOTOR_IND,GPIO.OUT)
GPIO.setup(MOTOR_INC,GPIO.OUT)
GPIO.setup(MOTOR_ENB,GPIO.OUT)
GPIO.output(MOTOR_IND,GPIO.LOW)
GPIO.output(MOTOR_INC,GPIO.LOW)

GPIO.setup(L_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(L_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(R_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(R_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

pA=GPIO.PWM(MOTOR_ENA,20)
pB=GPIO.PWM(MOTOR_ENB,20)
pA.start(0)
pB.start(0)

# Set initial encoder counts
left_count = 0
right_count = 0
left_distance = 0
right_distance = 0

# Define interrupt handlers for encoder signals
def encoder_callback_left(channel):
    global left_count, left_distance
    if GPIO.input(L_ENCODER_A) == GPIO.input(L_ENCODER_B):
        left_count += 1
    else:
        left_count -= 1
    left_distance = distance_traveled(left_count)


def encoder_callback_right(channel):
    global right_count, right_distance
    if GPIO.input(R_ENCODER_A) == GPIO.input(R_ENCODER_B):
        right_count += 1
    else:
        right_count -= 1
    right_distance = distance_traveled(right_count)


# Enable interrupt detection on encoder input pins
GPIO.add_event_detect(L_ENCODER_A, GPIO.BOTH, callback=encoder_callback_left)
GPIO.add_event_detect(L_ENCODER_B, GPIO.BOTH, callback=encoder_callback_left)
GPIO.add_event_detect(R_ENCODER_A, GPIO.BOTH, callback=encoder_callback_right)
GPIO.add_event_detect(R_ENCODER_B, GPIO.BOTH, callback=encoder_callback_right)


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
    forward()

if __name__ == "__main__":
    try:
        main()
 
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()