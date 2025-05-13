import RPi.GPIO as GPIO
import time
from time import sleep
import sys 
import math 

enA = 26
enB = 11
in1 = 19
in2 = 13
in3 = 6
in4 = 5
rotR= 2
rotL= 3



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

GPIO.setup(rotR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #rotary
GPIO.setup(rotL, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

pulseCountL = 0
pulseCountR = 0
offCountR = 0
offCountL = 0
eqDistL = 0
eqDistR = 0
wheelCircumference = 2 * 3 * 3; # Circumference (radius = 3 cm)



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
    
def right(speed):
    RIGHT.ChangeDutyCycle(speed)
    LEFT.ChangeDutyCycle(speed)

    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

def left(speed):
    RIGHT.ChangeDutyCycle(speed)
    LEFT.ChangeDutyCycle(speed)

    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    
def stop():
    RIGHT.ChangeDutyCycle(0)
    LEFT.ChangeDutyCycle(0)
    
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    
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

def opt2():
    global eqDistL,pulseCountL
    eqDistL = 0
    pulseCountL = 0
    
    dist = float(input("Enter diastance you wish to travel: ").strip())
    speed = int(input("Enter duty Cycle: ").strip())
    
    forward(speed)
    while eqDistL < dist:
        countPulse()
        print("Current Distance: ", eqDistL)
        time.sleep(0.1)
        
    stop()
    print("Target distance reached!")

#loop
def opt1():
    try: 
        while(1):
            sys.stdin.flush()
            
            x=input("Enter Command (f/b/r/l/s): ").strip().lower()
            speed=int(input("Enter duty Cycle: ").strip())
            timeDelay = float(input(" Enter excution time: "))
        
            
            sleep(0.1) 

            if x=='f':
                forward(speed)
                sleep(timeDelay)
                stop()
                
            elif x=='b':
                backward(speed)
                sleep(timeDelay)
                stop()
                
            elif x=='r':
                right(speed)
                sleep(timeDelay)
                stop()

            elif x=='l':
                left(speed)
                sleep(timeDelay)
                stop()
                
            elif x == 's':  # Stop command
                stop()
                
            elif x == 'e':  # Exit the program
                GPIO.cleanup()
                break
            else:
                print("Invalid input! Use f/b/r/l/s/e")
                
    except KeyboardInterrupt:
        print("\nProgram interrupted. Cleaning up GPIO...")
        GPIO.cleanup()

def main():
    while True: 
        print("Welcome to my world!!!")
        print("Option 1: Angle measurement")
        print("Option 2: Distance measurement")
        print("Option 3: Return to menu")
        
        choice = int(input("Enter (1 / 2 /3) to measure angle or distance: "))
        
        if choice == 1:
            opt1()
            main()
        elif choice == 2:
            opt2()
            main()
        elif choice == 3:
            GPIO.cleanup()
            print("Exiting...")
            break
        else:
            print("Invalud choice! Please enter 1,2, or 3.")
            
main()
