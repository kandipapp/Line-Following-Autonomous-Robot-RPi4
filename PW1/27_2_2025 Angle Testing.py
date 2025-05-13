import RPi.GPIO as GPIO
import time
from time import sleep
import sys

# GPIO Pin Setup
enA = 2
enB = 22
in1 = 3
in2 = 4
in3 = 17
in4 = 27

# Setup
GPIO.setmode(GPIO.BCM)

# Setup Motor Driver
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

# Turning time data (from experiment)
turn_times = {
    15: 0.15,
    30: 0.275,
    45: 0.4,
    60: 0.55,  # 30 + 30
    70: 0.65,  # 45 + 30
    75: 0.675, # 45 + 30
    90: 0.8,
    180: 1.44,
    270: 2.0,
    360: 2.75
}

def forward(time_duration):
    RIGHT.ChangeDutyCycle(70)
    LEFT.ChangeDutyCycle(70)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    sleep(time_duration)
    stop()

def backward(time_duration):
    RIGHT.ChangeDutyCycle(70)
    LEFT.ChangeDutyCycle(70)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    sleep(time_duration)
    stop()

def left(angle):
    if angle in turn_times:
        time_duration = turn_times[angle]
    else:
        print("Invalid angle. Choose a valid angle.")
        return
    
    RIGHT.ChangeDutyCycle(70)
    LEFT.ChangeDutyCycle(70)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    sleep(time_duration)
    stop()

def right(angle):
    if angle in turn_times:
        time_duration = turn_times[angle]
    else:
        print("Invalid angle. Choose a valid angle.")
        return
    
    RIGHT.ChangeDutyCycle(70)
    LEFT.ChangeDutyCycle(70)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    sleep(time_duration)
    stop()

def stop():
    RIGHT.ChangeDutyCycle(0)
    LEFT.ChangeDutyCycle(0)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

try:
    while True:
        command = input("Enter Command (f/b/l/r/s): ").strip().lower()
        if command in ['f', 'b']:
            time_duration = float(input("Enter execution time: ").strip())
            if command == 'f':
                forward(time_duration)
            elif command == 'b':
                backward(time_duration)
        elif command in ['l', 'r']:
            angle = int(input("Enter turning angle: ").strip())
            if command == 'l':
                left(angle)
            elif command == 'r':
                right(angle)
        elif command == 's':
            stop()
        elif command == 'e':
            GPIO.cleanup()
            break
        else:
            print("Invalid input! Use f/b/l/r/s/e")

except KeyboardInterrupt:
    print("\nProgram interrupted. Cleaning up GPIO...")
    GPIO.cleanup()
