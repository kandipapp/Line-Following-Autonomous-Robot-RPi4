import cv2
import numpy as np
import time
import os
from picamera2 import Picamera2
import RPi.GPIO as GPIO

# GPIO Setup
enA, enB = 2, 22
in1, in2, in3, in4 = 3, 4, 17, 27
rotR, rotL = 5, 6

GPIO.setmode(GPIO.BCM)
GPIO.setup([enA, enB, in1, in2, in3, in4], GPIO.OUT, initial=GPIO.LOW)
GPIO.setup([rotR, rotL], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

RIGHT, LEFT = GPIO.PWM(enA, 1000), GPIO.PWM(enB, 1000)
RIGHT.start(60)
LEFT.start(60)

# Initialize Pi Camera
camera = Picamera2()
camera.preview_configuration.main.size = (320, 240)
camera.preview_configuration.main.format = "RGB888"
camera.preview_configuration.controls.FrameRate = 60
camera.configure("preview")
camera.start()
time.sleep(2)

# Load reference images for special symbols
symbols_path = "/home/pi/Desktop/ImageSample"
symbols = {}
processed_symbols = {}

for filename in os.listdir(symbols_path):
    if filename.endswith((".png", ".jpg")):
        img = cv2.imread(os.path.join(symbols_path, filename), 0)
        symbols[filename] = img
        blur_img = cv2.GaussianBlur(img, (5, 5), 1.1)
        _, otsu_thresh = cv2.threshold(blur_img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        processed_symbols[filename] = otsu_thresh

# Motor Movement Functions
def forward():
    RIGHT.ChangeDutyCycle(40)
    LEFT.ChangeDutyCycle(40)
    GPIO.output([in1, in4], GPIO.LOW)
    GPIO.output([in2, in3], GPIO.HIGH)

def backward():
    RIGHT.ChangeDutyCycle(60)
    LEFT.ChangeDutyCycle(60)
    GPIO.output([in2, in3], GPIO.LOW)
    GPIO.output([in1, in4], GPIO.HIGH)

def left():
    RIGHT.ChangeDutyCycle(75)
    LEFT.ChangeDutyCycle(25)
    GPIO.output([in1, in4], GPIO.LOW)
    GPIO.output([in2, in3], GPIO.HIGH)

def right():
    RIGHT.ChangeDutyCycle(25)
    LEFT.ChangeDutyCycle(75)
    GPIO.output([in1, in4], GPIO.LOW)
    GPIO.output([in2, in3], GPIO.HIGH)

def left90():
    RIGHT.ChangeDutyCycle(100)
    LEFT.ChangeDutyCycle(100)
    GPIO.output([in2, in4], GPIO.HIGH)
    GPIO.output([in1, in3], GPIO.LOW)

def right90():
    RIGHT.ChangeDutyCycle(100)
    LEFT.ChangeDutyCycle(100)
    GPIO.output([in1, in3], GPIO.HIGH)
    GPIO.output([in2, in4], GPIO.LOW)

def stop():
    RIGHT.ChangeDutyCycle(0)
    LEFT.ChangeDutyCycle(0)
    GPIO.output([in1, in2, in3, in4], GPIO.LOW)

# Shape Identification Functions
def detect_shape(contour):
    approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
    num_vertices = len(approx)
    
    if num_vertices == 3:
        return "Triangle"
    elif num_vertices == 4:
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h
        return "Square" if 0.9 < aspect_ratio < 1.1 else "Rectangle"
    elif num_vertices > 6:
        return "Circle"
    return None

def classify_contour(contour):
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = float(w) / h
    area = cv2.contourArea(contour)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = area / hull_area if hull_area != 0 else 0

    if aspect_ratio > 3.5 and solidity < 0.5:
        return "Line"
    return "Symbol"

def detect_line_color(contour, frame):
    x, y, w, h = cv2.boundingRect(contour)
    roi = frame[y:y+h, x:x+w]
    avg_color = np.mean(roi, axis=(0, 1))

    if avg_color[2] > avg_color[1] and avg_color[2] > avg_color[0]:
        return "Red"
    elif avg_color[0] > avg_color[1] and avg_color[0] > avg_color[2]:
        return "Blue"
    return "Black"

# ORB Feature Matching for Special Symbols
orb = cv2.ORB_create(nfeatures=500)

def detect_with_orb(query_img, frame):
    kp1, des1 = orb.detectAndCompute(query_img, None)
    kp2, des2 = orb.detectAndCompute(frame, None)
    
    if des1 is None or des2 is None or len(des1) < 10 or len(des2) < 10:
        return 0
    
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    
    good_matches = [m for m in matches if m.distance < 20]
    return len(good_matches)

# Main Processing Function (Line Following + Symbol Detection)
def process_frame():
    frame = camera.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # Thresholding
    _, mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        shape_type = classify_contour(contour)
        x, y, w, h = cv2.boundingRect(contour)

        if shape_type == "Line":
            line_color = detect_line_color(contour, frame)
            print(f"Detected {line_color} line")
            
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)  # Get the largest contour (assumed to be the line)
                M = cv2.moments(c)
                cv2.drawContours(frame, [c], 0, (0, 255, 0), 3)
            

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
                cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

        elif shape_type == "Symbol":
            detected_shape = detect_shape(contour)
            if detected_shape:
                print(f"Detected shape: {detected_shape}")

            best_match, max_matches = None, 0
            for name, ref_img in processed_symbols.items():
                matches = detect_with_orb(ref_img, gray)
                if matches > max_matches:
                    max_matches = matches
                    best_match = name

            if best_match:
                print(f"Detected special symbol: {best_match}")

    cv2.imshow("Processed Frame", frame)

while True:
    process_frame()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
camera.stop()