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
    RIGHT.ChangeDutyCycle(30)
    LEFT.ChangeDutyCycle(30)
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
    LEFT.ChangeDutyCycle(75)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
 
def right90():
    global current_direction
    current_direction = "right90"
    RIGHT.ChangeDutyCycle(75)
    LEFT.ChangeDutyCycle(75)
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
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
 
 
def left90():
    global current_direction
    current_direction = "left90"
    RIGHT.ChangeDutyCycle(80)
    LEFT.ChangeDutyCycle(80)
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
                    right90()
 
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
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([130, 255, 150])
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
        final_mask = None
    elif cv2.countNonZero(mask_blue) > 500:
        color_detected = "Blue"
        final_mask = mask_blue
    elif cv2.countNonZero(mask_yellow) > 500:
        color_detected = "Yellow"
        final_mask = mask_yellow
    else:
        final_mask = None
 
    if final_mask is not None:
        if color_detected == "Blue":
            print("BLue Line -IGONORED")
            blue_mask_inv = cv2.bitwise_not(final_mask)  # Invert blue mask
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            gray_no_blue = cv2.bitwise_and(gray, gray, mask=blue_mask_inv)  # Mask out blue
            _, thresh = cv2.threshold(gray_no_blue, 100, 255, cv2.THRESH_BINARY_INV)
            mask_to_use = thresh
        else: 
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
 
def detect_with_template(template, image):
    result = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    return max_val, max_loc
 
 
def symbol_frame(frame,shape_contour):
    global current_direction  # Access direction control
    x_start, y_start = 180, 0
    x_end, y_end = 320, 120
    symbol_roi = frame[y_start:y_end, x_start:x_end] #[y_start:y_end, x_start:x_end]
 
    if current_direction == "forward":
        slowForward()
 
    gray_frame = cv2.cvtColor(symbol_roi, cv2.COLOR_BGR2GRAY)
    gray_frame = cv2.equalizeHist(gray_frame)
    blur_frame = cv2.GaussianBlur(gray_frame, (5,5), 1.1)
    _, otsu_frame = cv2.threshold(blur_frame, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
 
    cv2.imshow('Symbol', otsu_frame)
    best_match = None
    max_matches = 0
    min_match_count =10
 
    symbol_threshold = {
        'circle.jpg': 0.5,
        'hexagon.jpg': 0.1,
        'pentagon.jpg':0.1,
        'triangle.jpg': 0.5,
        'SemiquadCir.jpg': 0.2,
        'FaceRecognition': 0.7,
        'distanceGap.jpg': 0.7,
        'TrafficSign.jpg': 0.7,
        'up-arrow.jpg': 0.25,
        'down-arrow.jpg': 0.25,
        'left-arrow.jpg': 0.25,
        'right-arrow.jpg': 0.25,
        'STOPsign.jpg': 0.6,
    }
 
    for name, ref_img in processed_symbols.items():
        score, loc = detect_with_template(ref_img, otsu_frame)
        threshold = symbol_threshold.get(name, 0.7)  # Your custom value
 
        if score > max_matches and score >= threshold:
            max_matches = score
            best_match = name
 
    cv2.rectangle(frame,(x_start, y_start), (x_end, y_end),(0,255,0),2)
 
    print(f"Best match: {best_match}, Matches: {max_matches}")
 
    if best_match:
        ref_contours, _ = cv2.findContours(processed_symbols[best_match], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if ref_contours:
            ref_contour = max(ref_contours, key=cv2.contourArea)
            cv2.drawContours(symbol_roi, [ref_contour], -1, (255, 0, 0), 2)  # Draw on the ROI
 
        cv2.putText(frame, f"Symbol: {best_match} ({max_matches})", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
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