import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import os

# Initialize PiCamera2
camera = Picamera2()
camera.preview_configuration.main.size = (320, 240)
camera.preview_configuration.main.format = "RGB888"
camera.preview_configuration.controls.FrameRate = 30
camera.configure("preview")
camera.start()
time.sleep(2)  # Allow camera to warm up

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

def forward():
    RIGHT.ChangeDutyCycle(25)
    LEFT.ChangeDutyCycle(25)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def right():
    RIGHT.ChangeDutyCycle(20)
    LEFT.ChangeDutyCycle(50)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    
def right90():
    RIGHT.ChangeDutyCycle(70)
    LEFT.ChangeDutyCycle(70)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    
def left():
    RIGHT.ChangeDutyCycle(50)
    LEFT.ChangeDutyCycle(20)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    
def left90():
    RIGHT.ChangeDutyCycle(70)
    LEFT.ChangeDutyCycle(70)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    
def backward():
    RIGHT.ChangeDutyCycle(40)
    LEFT.ChangeDutyCycle(40)
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
    
def line_process_frame(line_frame):
  
    hsv = cv2.cvtColor(line_frame, cv2.COLOR_RGB2HSV)
    

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
        color_area = cv2.bitwise_and(line_frame, line_frame, mask=final_mask)
        gray_color = cv2.cvtColor(color_area, cv2.COLOR_RGB2GRAY)
        _, thresh = cv2.threshold(gray_color, 100, 255, cv2.THRESH_BINARY)
        mask_to_use = final_mask
    else:
        print("No color detected - following black line")
        gray = cv2.cvtColor(line_frame, cv2.COLOR_RGB2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        mask_to_use = thresh
        
    
    # Find contours on the chosen mask
    line_contours, _ = cv2.findContours(mask_to_use, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if line_contours:
        c = max(line_contours, key=cv2.contourArea)
        # Process the selected contour
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            print("CX:", cx, "CY:", cy)
            
            cv2.drawContours(line_frame, [c], -1, (0, 255, 0), 2)
            cv2.circle(line_frame, (cx, cy), 5, (255, 255, 255), -1)

            if cy > 170:
                if cx <= 120:
                    print("90 Left")
                    left90()
                elif 120 < cx < 200:
                    backward()
                else:
                    print("90 Right")
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

    # Optional: Display debugging windows
    cv2.imshow("Red Mask", mask_color)
    cv2.imshow("Line Mask", mask_to_use)
    cv2.imshow("Frame", line_frame)


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
        
for name, img in processed_symbols.items():
    cv2.imshow(name, img)  # Show the processed image
    cv2.waitKey(1000)  # Wait for 1 second (1000 ms)
    #cv2.destroyAllWindows()  # Close the window before showing the next image
    
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
    # If more than 6 edges, differentiate between Circle and Semiquad Circle
    elif num_vertices > 6:
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = w / float(h)

        hull = cv2.convexHull(contour, returnPoints=False)
        defects = cv2.convexityDefects(contour, hull)

        if defects is not None and len(defects) > 0:
            # Check for a large indentation (suggesting a semi-circle shape)
            for i in range(defects.shape[0]):
                _, _, _, depth = defects[i, 0]
                if depth > 10:  # Adjust threshold based on size
                    return "Circle"
        # If no major defects, assume it is a Circle
        return "Circle" 

    return None 

# Initialize ORB detector with more features for better accuracy
orb = cv2.ORB_create(nfeatures=500)  # Increased number of features

def detect_with_orb(query_img, frame):
    kp1, des1 = orb.detectAndCompute(query_img, None)
    kp2, des2 = orb.detectAndCompute(frame, None)

    if des1 is None or des2 is None or len(des1) < 10 or len(des2) < 10:
        return 0  # Not enough features for matching

    # BFMatcher with Hamming distance
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)

    # Apply threshold to filter only good matches
    good_matches = [m for m in matches if m.distance < 20]  # Lower distance means better match

    return len(good_matches)

while True:

    object_frame = camera.capture_array()
    line_process_frame(object_frame) 
    
    object_gray_frame = cv2.cvtColor(object_frame, cv2.COLOR_BGR2GRAY)
    object_blur_frame = cv2.GaussianBlur(object_gray_frame, (5,5), 1.1)
    _, object_otsu_frame = cv2.threshold(object_blur_frame, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    cv2.imshow('Symbol', object_otsu_frame)

    best_match = None
    max_matches = 0

    object_contours, hierarchy = cv2.findContours(object_otsu_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    detected_shape = None
    
    if object_contours:
        cnts = sorted(object_contours, key=cv2.contourArea, reverse = True) # Picks the contour
        if cnts:
            if len(object_contours) <2:
                cv2.drawContours(object_frame, [cnts[0]], -1, (0, 255, 0), 3)
                detected_shape = detect_shape(cnts[0])
                cv2.putText(object_frame, f"{detected_shape}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, (0, 255, 0), 2, cv2.LINE_AA)

            else:
                for name, ref_img in processed_symbols.items():
                    matches = detect_with_orb(ref_img, object_otsu_frame)  

                    if matches > max_matches:
                        max_matches = matches
                        best_match = name

                if best_match:
                    cv2.putText(object_frame, f"{best_match}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                                1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow('Symbol Detection', object_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.stop()
cv2.destroyAllWindows()
