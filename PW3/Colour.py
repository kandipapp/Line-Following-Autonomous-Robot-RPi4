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

RIGHT.start(60)
LEFT.start(60)

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
camera.preview_configuration.controls.FrameRate = 60
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
    LEFT.ChangeDutyCycle(60)
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
    RIGHT.ChangeDutyCycle(70)
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
    RIGHT.ChangeDutyCycle(80)
    LEFT.ChangeDutyCycle(80)
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

# Function to process the frame and determine movement
def process_frame():
    global frame
    frame = camera.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    

    # Define HSV ranges for each color
    # Red (example: tune as needed)
    lower_red1 = np.array([120, 180, 180])
    upper_red1 = np.array([130, 255, 255])

    # Green
    lower_green = np.array([35, 150, 100])
    upper_green = np.array([45, 255, 255])

    # Blue
    lower_blue = np.array([5, 50, 50])
    upper_blue = np.array([13, 255, 255])

    # Yellow
    lower_yellow = np.array([90, 100, 100])
    upper_yellow = np.array([100, 255, 255])

    # Create color masks
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) 
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # Combine all color masks
    mask_color = mask_red | mask_green | mask_blue | mask_yellow
    
    # Determine which color is detected (priority: red > green > blue > yellow)
    if cv2.countNonZero(mask_red) > 500:
        color_detected = "Red"
        final_mask = mask_red
    elif cv2.countNonZero(mask_green) > 500:
        color_detected = "Green"
        final_mask = mask_green
    elif cv2.countNonZero(mask_blue) > 500:
        color_detected = "Blue"
        final_mask = mask_blue
    elif cv2.countNonZero(mask_yellow) > 500:
        color_detected = "Yellow"
        final_mask = mask_yellow
    else:
        final_mask = None
        
    if final_mask is not None:
        print(f"{color_detected} color detected - following {color_detected} line")
        color_area = cv2.bitwise_and(frame, frame, mask=final_mask)
        gray_color = cv2.cvtColor(color_area, cv2.COLOR_RGB2GRAY)
        _, thresh = cv2.threshold(gray_color, 100, 255, cv2.THRESH_BINARY)
        mask_to_use = thresh
    else:
        print("No color detected - following black line")
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        mask_to_use = thresh
        
    
    # Find contours on the chosen mask
    contours, _ = cv2.findContours(mask_to_use, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        c = max(contours, key=cv2.contourArea)
        # Process the selected contour
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            print("CX:", cx, "CY:", cy)
            
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

            if cy > 170:
                if cx <= 140:
                    print("90 Left")
                    left90()
                elif 140 < cx < 170:
                    backward()
                else:
                    print("90 Right")
                    right90()
            else:
                if cx >= 240:
                    print("Right ->")
                    right()
                elif 120 < cx < 240:
                    print("Forward ^")
                    forward()
                else:
                    print("Left <-")
                    left()

            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

    # Optional: Display debugging windows
    cv2.imshow("Red Mask", mask_color)
    cv2.imshow("Black Mask", mask_to_use)
    cv2.imshow("Frame", frame)

while True:
    process_frame()  # Process each frame

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break

cv2.destroyAllWindows()
camera.stop()


