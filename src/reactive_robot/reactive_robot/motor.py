import RPi.GPIO as GPIO          
from time import sleep

in1 = 24
in2 = 23
in3 = 6
in4 = 5
enA = 12
enB = 13

DEFAULT_MOTOR_SPEED = 40

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

pA=GPIO.PWM(enA,20)
pB=GPIO.PWM(enB,20)
pA.start(0)
pB.start(0)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.HIGH)

GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.HIGH)

def forward():
    pA.ChangeDutyCycle(DEFAULT_MOTOR_SPEED)
    pB.ChangeDutyCycle(DEFAULT_MOTOR_SPEED)

def backward():
    pA.ChangeDutyCycle(DEFAULT_MOTOR_SPEED)
    pB.ChangeDutyCycle(DEFAULT_MOTOR_SPEED)


def turn_left():
    pA.ChangeDutyCycle(DEFAULT_MOTOR_SPEED*0.8)
    pB.ChangeDutyCycle(DEFAULT_MOTOR_SPEED*0.5)


def turn_right():
    pB.ChangeDutyCycle(DEFAULT_MOTOR_SPEED*0.8)
    pA.ChangeDutyCycle(DEFAULT_MOTOR_SPEED*0.5)

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





# print("\n")
# print("The default speed & direction of motor is LOW & Forward.....")
# print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
# print("\n")    

# while(1):

#     x=input()
    
#     if x=='r':
#         print("run")
#         if(temp1==1):
#          GPIO.output(in1,GPIO.HIGH)
#          GPIO.output(in2,GPIO.LOW)
#          GPIO.output(in3,GPIO.HIGH)
#          GPIO.output(in4,GPIO.LOW) 
#          print("forward")
#          x='z'
#         else:
#          GPIO.output(in1,GPIO.LOW)
#          GPIO.output(in2,GPIO.HIGH)
#          GPIO.output(in3,GPIO.LOW)
#          GPIO.output(in4,GPIO.HIGH)
#          print("backward")
#          x='z'


#     elif x=='s':
#         print("stop")
#         GPIO.output(in1,GPIO.LOW)
#         GPIO.output(in2,GPIO.LOW)
#         GPIO.output(in3,GPIO.LOW)
#         GPIO.output(in4,GPIO.LOW)
#         x='z'

#     elif x=='f':
#         print("forward")
#         GPIO.output(in1,GPIO.HIGH)
#         GPIO.output(in2,GPIO.LOW)
#         GPIO.output(in3,GPIO.HIGH)
#         GPIO.output(in4,GPIO.LOW)
#         temp1=1
#         x='z'

#     elif x=='b':
#         print("backward")
#         GPIO.output(in1,GPIO.LOW)
#         GPIO.output(in2,GPIO.HIGH)
#         temp1=0
#         x='z'

#     elif x=='l':
#         print("low")
#         pA.ChangeDutyCycle(25)
#         pB.ChangeDutyCycle(25)
#         x='z'

#     elif x=='m':
#         print("medium")
#         pA.ChangeDutyCycle(50)
#         pB.ChangeDutyCycle(50)
#         x='z'

#     elif x=='h':
#         print("high")
#         pA.ChangeDutyCycle(75)
#         pB.ChangeDutyCycle(75)
#         x='z'
     
    
#     elif x=='e':
#         GPIO.cleanup()
#         break
    
#     else:
#         print("<<<  wrong data  >>>")
#         print("please enter the defined data to continue.....")
