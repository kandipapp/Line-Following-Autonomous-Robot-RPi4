import RPi.GPIO as GPIO
import time

enA = 2
enB = 22
in1 = 3
in2 = 4
in3 = 17
in4 = 27
rotR= 5
rotL= 6

GPIO.setmode(GPIO.BCM)
GPIO.setup(enA,GPIO.OUT)
GPIO.setup(enB,GPIO.OUT)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)

A=GPIO.PWM(enA,1000)
B=GPIO.PWM(enB,1000)

A.start(0)
B.start(0)



def forward():
    A.ChangeDutyCycle(75)
    B.ChangeDutyCycle(75)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

def backward():
    A.ChangeDutyCycle(25)
    B.ChangeDutyCycle(25)
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
 
def right():
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

def left():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    
def stop():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    
while(1):
    
    x=input("Enter command (f/b/r/l/s): ").strip().lower()
    
    if x=='f':
        forward()
        
    elif x=='b':
        backward()
        
    elif x=='r':
        A.ChangeDutyCycle(25)
        B.ChangeDutyCycle(25)
        right()
        
    elif x=='l':
        A.ChangeDutyCycle(25)
        B.ChangeDutyCycle(25)
        left()
        
    elif x == 's':  # Stop command
        stop()
        
    elif x == 'e':  # Exit the program
        GPIO.cleanup()
        break
    
    else:
        print("Invalid input! Use f/b/r/l/s/e")