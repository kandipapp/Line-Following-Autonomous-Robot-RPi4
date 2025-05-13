
import RPi.GPIO as GPIO
import time
import numpy as np
import cv2
import os
from picamera2 import Picamera2
from collections import Counter
import threading
 
# ============================================================
# 1. GPIO / Motor Setup
# ============================================================
GPIO.setmode(GPIO.BCM)

ENA, ENB = 2, 22
IN1, IN2 = 3, 4
IN3, IN4 = 17, 27
 
GPIO.setup([ENA, ENB, IN1, IN2, IN3, IN4], GPIO.OUT)
 
pwmA = GPIO.PWM(ENA, 100)
pwmB = GPIO.PWM(ENB, 100)
pwmA.start(0)
pwmB.start(0)
 
def stop():
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
 
def drive(left_speed, right_speed):
    GPIO.output(IN1, GPIO.HIGH if left_speed >= 0 else GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW if left_speed >= 0 else GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW if right_speed >= 0 else GPIO.HIGH)
    GPIO.output(IN4, GPIO.HIGH if right_speed >= 0 else GPIO.LOW)
    pwmA.ChangeDutyCycle(min(abs(left_speed), 100))
    pwmB.ChangeDutyCycle(min(abs(right_speed), 100))
 
# ============================================================
# 2. PID Controller Setup
# ============================================================
Kp, Ki, Kd = 0.50, 0.0, 0.0
integral = 0
last_error = 0
last_time = time.time()
 
def compute_pid(error):
    global integral, last_error, last_time
    now = time.time()
    dt = max(now - last_time, 0.001)
    integral += error * dt
    derivative = (error - last_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    last_error = error
    last_time = now
    return output
 
# ============================================================
# 3. Camera Setup
# ============================================================
picam2 = Picamera2()
camera_config = picam2.create_video_configuration(main={"size": (160, 120)})
picam2.configure(camera_config)
picam2.start()
time.sleep(2)
 
FRAME_WIDTH = 160
FRAME_HEIGHT = 30
CENTER_TOLERANCE = 25  # pixels
 
# ============================================================
# 4. Color and Line Detection Configuration
# ============================================================
selected_colors = ["yellow", "red","blue","green"]
 
color_ranges = {
    "red": [
        
        (np.array([170, 100, 100]), np.array([185, 255, 255]))
    ],
    "blue": [
        (np.array([100, 100, 50]), np.array([115, 255, 255]))
    ],
    "green": [
        (np.array([75, 100, 100]), np.array([85, 255, 255]))
    ],
    "yellow": [
        (np.array([20, 100, 100]), np.array([30, 255, 255]))
    ],
}
 
black_lower = np.array([50, 0, 0])
black_upper = np.array([110, 50, 100])
 
BASE_SPEED = 10
STEER_LIMIT = 15

# ============================================================
# 5. Detection Variables and Functions
# ============================================================
arrow_templates = {}
arrow_result = None
shape_result = None
arrow_lock = threading.Lock()
shape_lock = threading.Lock()
last_detected_shape = None
last_detection_time = 0
 
def detect_shapes(frame):
    detected_shapes = []
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 1)
    edges = cv2.Canny(blur, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 300:
            approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
            sides = len(approx)
            perimeter = cv2.arcLength(cnt, True)
            circularity = 4 * np.pi * area / (perimeter ** 2 + 1e-6)
            shape = "Unknown"
            if sides == 3:
                shape = "Triangle"
            elif sides == 4:
                shape = "Rectangle"
            elif sides == 5:
                shape = "Pentagon"
            elif sides == 6:
                shape = "Hexagon"
            elif sides > 6:
                if circularity > 0.85:
                    shape = "Circle"
                elif 0.5 < circularity <= 0.7:
                    shape = "Partial Circle"
            x, y, w, h = cv2.boundingRect(cnt)
            x_center = x + w // 2
            if abs(x_center - FRAME_WIDTH // 2) < CENTER_TOLERANCE:
                detected_shapes.append((shape, x, y, w, h))
    return detected_shapes
 
def extract_blue_region(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([80, 30, 30])
    upper_blue = np.array([140, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        x_center = x + w // 2
        if abs(x_center - FRAME_WIDTH // 2) < CENTER_TOLERANCE:
            cropped = frame[y:y+h, x:x+w]
            return cropped
    return None
 
def detect_arrow_template(cropped, templates):
    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    resized = cv2.resize(gray, (100, 100))
    best_score = -1
    best_label = None
    for label, tpl in templates.items():
        result = cv2.matchTemplate(resized, tpl, cv2.TM_CCOEFF_NORMED)
        _, score, _, _ = cv2.minMaxLoc(result)
        if score > best_score:
            best_score = score
            best_label = label
    return best_label if best_score >= 0.6 else None
 
def load_arrow_templates(template_dir):
    templates = {}
    directions = ['left', 'right', 'up', 'down']
    shapes = ['circle', 'rect']
    for d in directions:
        for s in shapes:
            filename = f"{d}_{s}.jpg"
            path = os.path.join(template_dir, filename)
            img = cv2.imread(path, 0)
            if img is not None:
                resized = cv2.resize(img, (100, 100))
                templates[f"{d.upper()} ({s})"] = resized
    return templates
 
arrow_templates = load_arrow_templates("/home/pi/Desktop/ImageSample")
last_detected_arrow = None
last_arrow_detection_time = 0
 
 
 
# ============================================================
# 6. Main Loop
# ============================================================
try:
    while True:
        frame_rgb = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        frame_bottom = frame_bgr[90:120, :]
        hsv = cv2.cvtColor(frame_bottom, cv2.COLOR_BGR2HSV)

        symbol_detected = False  # Flag to skip shape if arrow is found

        current_time = time.time()
        if current_time - last_arrow_detection_time > 3:
            cropped_arrow = extract_blue_region(frame_bgr)
            if cropped_arrow is not None:
                arrow = detect_arrow_template(cropped_arrow, arrow_templates)
                if arrow and arrow != "Unknown" and (current_time - last_detection_time > 5):
                    stop()
                    print(f"Detected Arrow: {arrow}")
                    print("Sampling arrow detections for 6 seconds...")
                    start_time = time.time()
                    arrow_results = []
                    while time.time() - start_time < 6:
                        frame_rgb = picam2.capture_array()
                        stable_frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                        cropped_arrow = extract_blue_region(stable_frame)
                        if cropped_arrow is not None:
                            result = detect_arrow_template(cropped_arrow, arrow_templates)
                            if result:
                                arrow_results.append(result)
                        time.sleep(0.3)
                    print("Arrow Results:", arrow_results)

                    print("Repeating sampling...")
                    start_time = time.time()
                    arrow_results_repeat = []
                    while time.time() - start_time < 6:
                        frame_rgb = picam2.capture_array()
                        stable_frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                        cropped_arrow = extract_blue_region(stable_frame)
                        if cropped_arrow is not None:
                            result = detect_arrow_template(cropped_arrow, arrow_templates)
                            if result:
                                arrow_results_repeat.append(result)
                        time.sleep(0.3)
                    print("Second Arrow Results:", arrow_results_repeat)

                    if arrow_results_repeat:
                        most_common_arrow = Counter(arrow_results_repeat).most_common(1)[0][0]
                        print("Most Frequent Arrow Detected:", most_common_arrow)

                    last_detected_arrow = arrow
                    last_arrow_detection_time = time.time()
                    last_detection_time = last_arrow_detection_time
                    print("Resuming after arrow detection...")
                    drive(40, 40)
                    time.sleep(0.5)
                    drive(BASE_SPEED, BASE_SPEED)
                    
                    symbol_detected = True 
        if not symbol_detected: 
            current_time = time.time()
            detected_shapes = detect_shapes(frame_bgr)
            shape = detected_shapes[0][0] if detected_shapes else None
            if shape and shape != "Unknown" and (current_time - last_detection_time > 5):
                stop()
                print(f"Detected Shape: {shape}")
                print("Sampling shape detections for 6 seconds...")
                start_time = time.time()
                shape_results = []
                while time.time() - start_time < 6:
                    frame_rgb = picam2.capture_array()
                    stable_frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                    detected_shapes = detect_shapes(stable_frame)
                    if detected_shapes:
                        shape_results.append(detected_shapes[0][0])
                    time.sleep(0.3)
                print("Shape Results:", shape_results)
                print("Repeating sampling...")
                start_time = time.time()
                shape_results_repeat = []
                while time.time() - start_time < 6:
                    frame_rgb = picam2.capture_array()
                    stable_frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                    detected_shapes = detect_shapes(stable_frame)
                    if detected_shapes:
                        shape_results_repeat.append(detected_shapes[0][0])
                    time.sleep(0.3)
                print("Second Shape Results:", shape_results_repeat)
                if shape_results_repeat:
                    most_common_shape = Counter(shape_results_repeat).most_common(1)[0][0]
                    print("Most Frequent Shape Detected:", most_common_shape)
                last_detected_shape = shape
                last_arrow_detection_time = time.time()
                last_detection_time = last_arrow_detection_time
                print("Resuming after shape detection...")
                drive(40, 40)
                time.sleep(0.2)
                drive(BASE_SPEED, BASE_SPEED)
     
            mask = cv2.inRange(hsv, black_lower, black_upper)
            target_color = "black"
            for color in selected_colors:
                mask_color = None
                for (lower, upper) in color_ranges[color]:
                    temp_mask = cv2.inRange(hsv, lower, upper)
                    mask_color = temp_mask if mask_color is None else cv2.bitwise_or(mask_color, temp_mask)
                mask_color = cv2.erode(mask_color, None, iterations=1)
                mask_color = cv2.dilate(mask_color, None, iterations=1)
                if cv2.countNonZero(mask_color) > 50:
                    mask = mask_color
                    target_color = color
                    break
            mask = cv2.erode(mask, None, iterations=1)
            mask = cv2.dilate(mask, None, iterations=1)
     
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    error = cx - FRAME_WIDTH // 2
                    pid_output = compute_pid(error)
                    offset = max(min(pid_output, STEER_LIMIT), -STEER_LIMIT)
                    left_speed = BASE_SPEED - offset
                    right_speed = BASE_SPEED + offset
                    drive(left_speed, right_speed)
                else:
                    drive(-80, -80)
            else:
                drive(-80, -80)
     
            cv2.imshow("LineFollow", frame_bottom)
            cv2.imshow("Shape Detection", frame_bgr)
            cv2.waitKey(1)
 
except KeyboardInterrupt:
    print("Stopped by user.")
 
finally:
    stop()
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()