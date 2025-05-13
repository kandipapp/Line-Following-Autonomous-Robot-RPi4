import RPi.GPIO as GPIO
import time
from time import sleep
import sys 
import math 

# Motor Driver Pins
enA = 2
enB = 22
in1 = 3
in2 = 4
in3 = 17
in4 = 27

# Rotary Encoder Pins
rotR = 5
rotL = 6

# Setup
GPIO.setmode(GPIO.BCM)

# Motor Driver Setup
GPIO.setup(enA, GPIO.OUT)
GPIO.setup(enB, GPIO.OUT)

GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)

GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)

RIGHT = GPIO.PWM(enA, 1000)  # Frequency = 1000Hz
LEFT = GPIO.PWM(enB, 1000)

RIGHT.start(0)
LEFT.start(0)

# Rotary Encoder Setup
GPIO.setup(rotR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(rotL, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Variables
pulseCountL = 0
pulseCountR = 0
offCountR = 0
offCountL = 0
eqDistL = 0
eqDistR = 0
wheelCircumference = 2 * math.pi * 3  # Using p for better accuracy

# Motor Control Functions
def forward(speed):
    RIGHT.ChangeDutyCycle(speed)
    LEFT.ChangeDutyCycle(speed)

    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def backward(speed):
    RIGHT.ChangeDutyCycle(speed)
    LEFT.ChangeDutyCycle(speed)

    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    
def right(speed):
    RIGHT.ChangeDutyCycle(speed)
    LEFT.ChangeDutyCycle(speed)

    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def left(speed):
    RIGHT.ChangeDutyCycle(speed)
    LEFT.ChangeDutyCycle(speed)

    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

def stop():
    RIGHT.ChangeDutyCycle(0)
    LEFT.ChangeDutyCycle(0)

    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    
# Rotary Encoder Pulse Counting
def countPulse(channel):
    global pulseCountR, pulseCountL, eqDistR, eqDistL, offCountR, offCountL

    if channel == rotR:
        pulseCountR += 1
        eqDistR = (pulseCountR * wheelCircumference / 40)
    elif channel == rotL:
        pulseCountL += 1
        eqDistL = (pulseCountL * wheelCircumference / 40)

# Attach Interrupts for Rotary Encoder with Debouncing
GPIO.add_event_detect(rotR, GPIO.RISING, callback=countPulse, bouncetime=10)
GPIO.add_event_detect(rotL, GPIO.RISING, callback=countPulse, bouncetime=10)

# Distance Measurement
def opt2():
    global eqDistL, eqDistR, pulseCountL, pulseCountR
    eqDistL = eqDistR = 0
    pulseCountL = pulseCountR = 0
    
    dist = float(input("Enter distance to travel: ").strip())
    speed = int(input("Enter duty cycle: ").strip())
    
    forward(speed)
    
    while (eqDistL + eqDistR) / 2 < dist:  # Average of both wheels
        print(f"Current Distance: {(eqDistL + eqDistR) / 2:.2f} cm")
        time.sleep(0.1)
    
    stop()
    print("Target distance reached!")

# Manual Control Loop
def opt1():
    try: 
        while True:
            x = input("Enter Command (f/b/r/l/s/e): ").strip().lower()
            if x == 'e':  # Exit the program
                GPIO.cleanup()
                break
            elif x in ('f', 'b', 'r', 'l', 's'):
                speed = int(input("Enter duty cycle: ").strip())
                timeDelay = float(input("Enter execution time: ").strip())
                sleep(0.1) 

                if x == 'f':
                    forward(speed)
                elif x == 'b':
                    backward(speed)
                elif x == 'r':
                    right(speed)
                elif x == 'l':
                    left(speed)
                elif x == 's':
                    stop()

                sleep(timeDelay)
                stop()
            else:
                print("Invalid input! Use f/b/r/l/s/e")
    except KeyboardInterrupt:
        print("\nProgram interrupted. Cleaning up GPIO...")
        GPIO.cleanup()

# Main Menu
def main():
    try:
        while True: 
            print("\nWelcome to my world!!!")
            print("1: Manual Control")
            print("2: Distance Measurement")
            print("3: Exit")
            
            choice = int(input("Enter (1 / 2 / 3): ").strip())
            
            if choice == 1:
                opt1()
            elif choice == 2:
                opt2()
            elif choice == 3:
                GPIO.cleanup()
                print("Exiting...")
                break
            else:
                print("Invalid choice! Please enter 1, 2, or 3.")
    except KeyboardInterrupt:
        print("\nProgram interrupted. Cleaning up GPIO...")
        GPIO.cleanup()

main()

