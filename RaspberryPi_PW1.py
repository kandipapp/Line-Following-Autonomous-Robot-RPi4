import RPi.GPIO as GPIO
import time
from time import sleep
import sys 
import math 

enA = 2
enB = 22
in1 = 3
in2 = 4
in3 = 17
in4 = 27
rotR= 5
rotL= 6

#setup 
GPIO.setmode(GPIO.BCM)

#setup Motor Driver 
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

RIGHT = GPIO.PWM(enA,1000) #frequency = 1000Hz 
LEFT = GPIO.PWM(enB,1000)

RIGHT.start(0)
LEFT.start(0)


def forward(speed):
    RIGHT.ChangeDutyCycle(speed)
    LEFT.ChangeDutyCycle(speed)

    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

def backward(speed):
    RIGHT.ChangeDutyCycle(speed)
    LEFT.ChangeDutyCycle(speed)

    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

def stop():
    RIGHT.ChangeDutyCycle(0)
    LEFT.ChangeDutyCycle(0)
    
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)

def right_15():
    RIGHT.ChangeDutyCycle(65)
    LEFT.ChangeDutyCycle(65)

    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

    sleep(0.25) 

def right_30():
    RIGHT.ChangeDutyCycle(69)
    LEFT.ChangeDutyCycle(69)

    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

    sleep(0.3) 

def right_45():
    RIGHT.ChangeDutyCycle(68)
    LEFT.ChangeDutyCycle(68)

    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

    sleep(0.4) 

def right_90():
    RIGHT.ChangeDutyCycle(70)
    LEFT.ChangeDutyCycle(70)

    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

    sleep(0.75) 

def right_180():
    RIGHT.ChangeDutyCycle(73)
    LEFT.ChangeDutyCycle(73)

    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

    sleep(1.3) 

def right_360():
    RIGHT.ChangeDutyCycle(100)
    LEFT.ChangeDutyCycle(100)

    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

    sleep(1.63) 

def left_15():
    RIGHT.ChangeDutyCycle(50)
    LEFT.ChangeDutyCycle(50)

    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

    sleep(0.25)
    
def left_30():
    RIGHT.ChangeDutyCycle(53)
    LEFT.ChangeDutyCycle(53)

    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

    sleep(0.5)

def left_45():
    RIGHT.ChangeDutyCycle(50)
    LEFT.ChangeDutyCycle(50)

    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

    sleep(0.65)

def left_90():
    RIGHT.ChangeDutyCycle(64)
    LEFT.ChangeDutyCycle(64)

    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

    sleep(1)

def left_180():
    RIGHT.ChangeDutyCycle(73)
    LEFT.ChangeDutyCycle(73)

    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

    sleep(1.3)    

def left_360():
    RIGHT.ChangeDutyCycle(100)
    LEFT.ChangeDutyCycle(100)

    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

    sleep(1.68)




def countPulse():
    global pulseCountR, pulseCountL, eqDistR, eqDistL, offCountR, offCountL
    onCountR = GPIO.input(rotR)
    onCountL = GPIO.input(rotL)

    if offCountR != onCountR:
        pulseCountR += 1
        eqDistR = (pulseCountR*wheelCircumference/40)
        offCountR = onCountR

    if offCountL != onCountL:
        pulseCountL += 1
        eqDistL =(pulseCountL*wheelCircumference/40)
        offCountL = onCountL

#loop
try: 
    while(1):
        sys.stdin.flush()
        
        direction=input("Enter Command (f/b/r/l/s): ").strip().lower()
        countPulse()
        
        sleep(0.1) 

        if direction=='f':
            speed=int(input("Enter duty Cycle: ").strip())
            forward(speed)
            sleep(2)
            stop()
            
        elif direction=='b':
            speed=int(input("Enter duty Cycle: ").strip())
            backward(speed)
            sleep(2)
            stop()
            
        elif direction=='r':
            degree_R = int(input("Enter degree of right turn: ").strip())

            right()
        
            stop()

        elif direction=='l':
            degree_L = int(input("Enter degree of right turn: ").strip())
            left()
      
            stop()
            
        elif direction == 's':  # Stop command
            stop()
            
        elif direction == 'e':  # Exit the program
            GPIO.cleanup()
            break
        else:
            print("Invalid input! Use f/b/r/l/s/e")
            
except KeyboardInterrupt:
    print("\nProgram interrupted. Cleaning up GPIO...")
    GPIO.cleanup()
