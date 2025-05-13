import cv2
import numpy as np
import time
from picamera2 import Picamera2
import RPi.GPIO as GPIO
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

GPIO.setup(rotR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #rotary
GPIO.setup(rotL, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

pulseCountL = 0
pulseCountR = 0
offCountR = 0
offCountL = 0
eqDistL = 0
eqDistR = 0
wheelCircumference = 2 * 3 * 3; # Circumference (radius = 3 cm)

# Initialize Pi Camera
camera = Picamera2()
camera.preview_configuration.main.size = (320, 240)
camera.preview_configuration.main.format = "RGB888"
camera.preview_configuration.controls.FrameRate = 30
camera.configure("preview")
camera.start()

def forward():
    RIGHT.ChangeDutyCycle(40)
    LEFT.ChangeDutyCycle(40)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def right():
    RIGHT.ChangeDutyCycle(25)
    LEFT.ChangeDutyCycle(75)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    
def right90():
    RIGHT.ChangeDutyCycle(90)
    LEFT.ChangeDutyCycle(90)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    
def left():
    RIGHT.ChangeDutyCycle(75)
    LEFT.ChangeDutyCycle(25)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    
def left90():
    RIGHT.ChangeDutyCycle(100)
    LEFT.ChangeDutyCycle(100)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    
def backward():
    RIGHT.ChangeDutyCycle(60)
    LEFT.ChangeDutyCycle(60)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

def stop():
    RIGHT.ChangeDutyCycle(0)
    LEFT.ChangeDutyCycle(0)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    
    
def countPulse():
    global pulseCountR, pulseCountL, eqDistR, eqDistL, offCountR, offCountL
    onCountR = GPIO.input(rotR)
    onCountL = GPIO.input(rotL)

    if offCountR != onCountR:
        pulseCountR += 1
        eqDistR = (pulseCountR*wheelCircumference/40.0)
        offCountR = onCountR

    if offCountL != onCountL:
        pulseCountL += 1
        eqDistL =(pulseCountL*wheelCircumference/40.0)
        offCountL = onCountL
        
        
# Function to process the frame and determine movement
def process_frame():
    frame = camera.capture_array()  # CapAture image from PiCamera
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)  # Convert to grayscale
    _, mask = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)  # Thresholding

    # Find contours of the black line
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)  # Get the largest contour (assumed to be the line)
        M = cv2.moments(c)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])  # Centroid X
            cy = int(M["m01"] / M["m00"])  # Centroid Y
            print("CX:", cx, "CY:", cy)

            # Determine robot movement
            if (cy>170):
                if (cx <= 140):
                    print(" 90 Left ")
                    left90()
                elif (140< cx < 170):
                    backward()
                else :
                    print("90 Right ")
                    right90()
            else:
                if (cx >=240):
                    print("Right ->")
                    right()
                elif (120 < cx < 240):
                    print("Forward ^")
                    forward() 
                else:
                    print("Left <-")
                    left() 

            # Draw centroid
            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

    # Show processed images
    cv2.imshow("Mask", mask)
    cv2.imshow("Frame", frame)

distanceStop = int(input("Enter Distance you wish to stop for 2s :"))
stopTime =0
paused = False


while True:
    countPulse()
    print(f"distance = {eqDistR}")
    
    if not paused:  # If the robot is moving
        if eqDistR >= distanceStop:  # Stop condition
            stop()
            print(f"Stopped at {eqDistR} cm, waiting for 2 seconds...")
            stopTime = time.time()  # Record stop time
            paused = True  # Set pause flag
        else:
            process_frame()  # Keep following the line

    else:  # If the robot is paused
        if time.time() - stopTime >= 2:  # Check if 2 seconds have passed
            print("Resuming movement...")
            process_frame()


    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break


cv2.destroyAllWindows()
camera.stop()
GPIO.cleanup()
