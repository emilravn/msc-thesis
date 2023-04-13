import RPi.GPIO as GPIO          
from time import sleep

in1 = 24
in2 = 23
in3 = 6
in4 = 5
enA = 12
enB = 13

DEFAULT_MOTOR_SPEED = 30

GPIO.setmode(GPIO.BCM)

GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(enA,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)

GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(enB,GPIO.OUT)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)

pA=GPIO.PWM(enA,15)
pB=GPIO.PWM(enB,15)
pA.start(0)
pB.start(0)

def forward():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)

    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    pA.ChangeDutyCycle(DEFAULT_MOTOR_SPEED)
    pB.ChangeDutyCycle(DEFAULT_MOTOR_SPEED)

def backward():
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)

    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
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
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    pA.stop()
    pB.stop()
    GPIO.cleanup()
