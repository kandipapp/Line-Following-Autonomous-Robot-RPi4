import cv2
import numpy as np
import time
from picamera2 import Picamera2
import RPi.GPIO as GPIO
from time import sleep
import sys
import math
import os
 
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
current_direction = "stop"  # Default
 
# Initialize Pi Camera
camera = Picamera2()
camera.preview_configuration.main.size = (320, 240)
camera.preview_configuration.main.format = "RGB888"
camera.preview_configuration.controls.FrameRate = 60
camera.configure("preview")
camera.start()
# For line following
ROI_Y_START = 120
ROI_Y_END = 240
 
 
# Load reference images (store multiple orientations if necessary)
symbols_path = "/home/pi/Desktop/ImageSample"
symbols = {}
processed_symbols = {}  # Store preprocessed images
 
for filename in os.listdir(symbols_path):
    if filename.endswith(".png") or filename.endswith(".jpg"):
        img_path = os.path.join(symbols_path, filename)
        img = cv2.imread(img_path, 0)
        symbols[filename] = img
 
        blur_img = cv2.GaussianBlur(img, (5, 5), 1.1)
        _, otsu_thresh = cv2.threshold(blur_img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
 
        processed_symbols[filename] = otsu_thresh  # Store processed image
 
#for name, img in processed_symbols.items():
#    cv2.imshow(name, img)  # Show the processed image
#    cv2.waitKey(1000)  # Wait for 1 second (1000 ms)
    #cv2.destroyAllWindows()  # Close the window before showing the next image
 
 
def forward():
    global current_direction
    current_direction = "forward"
    RIGHT.ChangeDutyCycle(27)
    LEFT.ChangeDutyCycle(27)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
 
def slowForward():
    global current_direction
    current_direction = "forward"
    RIGHT.ChangeDutyCycle(25)
    LEFT.ChangeDutyCycle(25)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
 
def right():
    global current_direction
    current_direction = "right"
    RIGHT.ChangeDutyCycle(25)
    LEFT.ChangeDutyCycle(60)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
 
def rightback():
    global current_direction
    current_direction = "rightback"
    RIGHT.ChangeDutyCycle(25)
    LEFT.ChangeDutyCycle(70)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
 
def right90():
    global current_direction
    current_direction = "right90"
    RIGHT.ChangeDutyCycle(65)
    LEFT.ChangeDutyCycle(65)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
 
def left():
    global current_direction
    current_direction = "left"
    RIGHT.ChangeDutyCycle(70)
    LEFT.ChangeDutyCycle(25)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
 
def leftback():
    global current_direction
    current_direction = "leftback"
    RIGHT.ChangeDutyCycle(55)
    LEFT.ChangeDutyCycle(27)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
 
 
def left90():
    global current_direction
    current_direction = "left90"
    RIGHT.ChangeDutyCycle(70)
    LEFT.ChangeDutyCycle(70)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
 
def backward():
    global current_direction
    current_direction = "backward"
    RIGHT.ChangeDutyCycle(45)
    LEFT.ChangeDutyCycle(45)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
 
def stop():
    global current_direction
    current_direction = "stop"
    RIGHT.ChangeDutyCycle(0)
    LEFT.ChangeDutyCycle(0)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
 
def detect_shape(contour):
    approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
    num_vertices = len(approx)
 
    if num_vertices == 3:
        return "Triangle"
    elif num_vertices == 4:
        return "Rectangle"
    elif num_vertices == 5:
        return "Pentagon"
    elif num_vertices == 6:
        return "Hexagon"
    elif num_vertices > 6:
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = w / float(h)
        circularity = (4 * math.pi * cv2.contourArea(contour)) / (cv2.arcLength(contour, True) ** 2)
 
        defects = None  # <-- initialize first
 
        hull = cv2.convexHull(contour, returnPoints=False)
        if len(contour) >= 3 and hull is not None and len(hull) >= 3:
            try:
                defects = cv2.convexityDefects(contour, hull)
            except cv2.error as e:
                print("ConvexityDefects error:", e)
 
        if defects is not None and len(defects) > 0:
            for i in range(defects.shape[0]):
                _, _, _, depth = defects[i, 0]
                if depth > 10:
                    return None
        elif circularity >0.7:
            return "Circle"
 
        return None
 
    return None
 
# Initialize ORB detector with more features for better accuracy
orb = cv2.ORB_create(nfeatures=500)  # Increased number of features
 
def detect_with_orb(query_img, roi_symbol):
    kp1, des1 = orb.detectAndCompute(query_img, None)
    kp2, des2 = orb.detectAndCompute(roi_symbol, None)
 
    if des1 is None or des2 is None or len(des1) < 20 or len(des2) < 20:
        return 0
 
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = bf.knnMatch(des1, des2, k=2)
 
    good = [m for m, n in matches if m.distance < 0.75 * n.distance]
    return len(good)
 
def movement(frame, line_contour):
    M = cv2.moments(line_contour)
    if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            print("CX:", cx, "CY:", cy)
 
            cv2.drawContours(frame, [line_contour], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
 
            if cy > 185:
                if cx <= 140:
                    print("90 Left")
                    rightback
                    left90()
                elif 150 < cx < 170:
                    backward()
                else:
                    print("90 Right")
                    leftback()
                    right90()
            else:
                if cx >= 200:
                    print("Right ->")
                    right()
                elif 120 < cx < 200:
                    print("Forward ^")
                    forward()
                else:
                    print("Left <-")
                    left()
 
# Function to process the frame and determine movement
def process_frame():
    frame = camera.capture_array()  # Capture image from PiCamera
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
 
    # Define HSV color ranges (tweak as needed)
    lower_red1 = np.array([170, 100, 100])
    upper_red1 = np.array([185, 255, 255])
    lower_green = np.array([75, 100, 100])
    upper_green = np.array([85, 255, 255])
    lower_blue = np.array([100, 100, 50])
    upper_blue = np.array([115, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
 
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
        mask_to_use = final_mask
    else:
        print("No color detected - following black line")
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        mask_to_use = thresh
 
 
    # Find contours on the chosen mask
    contours, _ = cv2.findContours(mask_to_use, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
 
 
    if len(contours) >= 2:
        # Largest contour is the path line
        line_contour = contours[0]
        shape_contour = contours[1]
 
        # Line following logic (based on largest contour)
        movement(frame, line_contour)
        if current_direction == "forward":
            symbol_frame(frame, shape_contour)
 
            if len(contours) ==2:
                shape_name = detect_shape(shape_contour)
 
                if shape_name:
                    cv2.drawContours(frame, [shape_contour], -1, (255, 0, 0), 2)
                   # cv2.putText(frame, shape_name, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                   #        1, (0, 255, 0), 2, cv2.LINE_AA)
                elif shape_name in["Rectangle","Circle"]:
                    symbol_frame(symbol_roi, shape_contour)
 
    elif len(contours) ==1:
        cv2.putText(frame, "-", (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        movement(frame, contours[0])
 
    elif len(contours) ==0:
        print("No Contours")
        backward()
 
    # Show processed images
    cv2.imshow("Mask", mask_to_use)
    cv2.imshow("Frame", frame)
 
def symbol_frame(frame,shape_contour):
    x_start, y_start = 100, 0
    x_end, y_end = 320, 120
    symbol_roi = frame[y_start:y_end, x_start:x_end]
 
    gray_frame = cv2.cvtColor(symbol_roi, cv2.COLOR_BGR2GRAY)
    gray_frame = cv2.equalizeHist(gray_frame)
    blur_frame = cv2.GaussianBlur(gray_frame, (5,5), 1.1)
    _, otsu_frame = cv2.threshold(blur_frame, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
 
    cv2.imshow('Symbol', otsu_frame)
    best_match = None
    max_matches = 0
    min_match_count =1
 
    for name, ref_img in processed_symbols.items():
        matches = detect_with_orb(ref_img, otsu_frame)  
 
        if matches > max_matches:
            max_matches = matches
            best_match = name
 
    cv2.rectangle(frame,(x_start, y_start), (x_end, y_end),(0,255,0),2)
 
    print(f"Best match: {best_match}, Matches: {max_matches}")
 
    if best_match and max_matches >= min_match_count:
        cv2.putText(frame, f"Symbol: {best_match} ({max_matches})", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('Symbol Detection', frame)
 
    else:
         # If ORB fails, fallback to contour-based shape detection
        contours, _ = cv2.findContours(otsu_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            detected_shape = detect_shape(largest_contour)
 
            if detected_shape:
                cv2.drawContours(symbol_roi, [largest_contour], -1, (255, 0, 0), 2)
                cv2.putText(frame, f"Shape: {detected_shape}", (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
 
        cv2.imshow('Symbol Detection', frame)
 
try:
    while True:
        process_frame()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    stop()
    camera.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()