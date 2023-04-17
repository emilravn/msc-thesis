import RPi.GPIO as GPIO          
from time import sleep

MOTOR_INA = 24
MOTOR_INB = 23
MOTOR_IND = 6
MOTOR_INC = 5
MOTOR_ENA = 12
MOTOR_ENB = 13

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

pA=GPIO.PWM(MOTOR_ENA,15)
pB=GPIO.PWM(MOTOR_ENB,15)
pA.start(0)
pB.start(0)

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
