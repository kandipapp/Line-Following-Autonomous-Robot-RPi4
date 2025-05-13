import cv2
import numpy as np
import time
from picamera2 import Picamera2

# Initialize Pi Camera
camera = Picamera2()
camera.preview_configuration.main.size = (320, 240)
camera.preview_configuration.main.format = "RGB888"
camera.preview_configuration.controls.FrameRate = 30
camera.configure("preview")
camera.start()

# Function to process the frame and determine movement
def process_frame():
    frame = camera.capture_array()  # Capture image from PiCamera
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
            if cx >= 240:
                print("Turn Left")
            elif 130 < cx < 240:
                print("Move Forward")
            else:
                print("Turn Right")

            # Draw centroid
            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

    # Show processed images
    cv2.imshow("Mask", mask)
    cv2.imshow("Frame", frame)

# Function to detect symbols like circles, 3/4 circles, and arrows
def detect_symbol():
    frame = camera.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)  # Convert to grayscale
    blurred = cv2.GaussianBlur(gray, (5,5), 0)  # FIXED GaussianBlur spelling

    edges = cv2.Canny(gray, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Detect circles using Hough Transform
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30, 
                               param1=50, param2=30, minRadius=10, maxRadius=100)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            x, y, r = i[0], i[1], i[2]

            h, w = gray.shape
            x1, y1 = max(0, x - r), max(0, y - r)
            x2, y2 = min(w, x + r), min(h, y + r)

            roi = gray[y1:y2, x1:x2]

            if roi.size == 0:
                print("ROI is empty, skipping...")
                continue  # Skip this iteration

            edges = cv2.Canny(roi, 50, 150)


    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)

        # Detect rectangle
        if len(approx) == 4:
            print("Rectangle detected!")
            x, y, w, h = cv2.boundingRect(approx)
            roi = gray[y:y+h, x:x+w]
            detected_symbol = detect_specSymbol(roi)

            if detected_symbol:
                print(f"Detected Symbol: {detected_symbol}")

        elif len(approx) == 3:
            print("Triangle detected!")
        elif len(approx) == 5:
            print("Pentagon detected!")
        elif len(approx) == 6:
            print("Heptagon detected!")

# Function to detect arrows inside circles
def detect_specSymbol(roi):
    edges = cv2.Canny(roi, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)

        # Check if shape is a triangle (arrowhead)
        if len(approx) == 3:
            (x, y, w, h) = cv2.boundingRect(approx)

            # Check arrow direction
            if y < roi.shape[0] // 2:  
                print("Detected an UP Arrow inside Circle!")
            else:  
                print("Detected a DOWN Arrow inside Circle!")

    # Detect Left/Right Arrows using contours
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        if 7 <= len(approx) <= 10:  # If shape has 7-10 corners, likely an arrow
            moments = cv2.moments(contour)
            if moments["m00"] != 0:
                cx = int(moments["m10"] / moments["m00"])

                if cx < roi.shape[1] * 0.4:
                    print("Detected LEFT Arrow!")
                    return "LEFT_ARROW"
                elif cx > roi.shape[1] * 0.6:
                    print("Detected RIGHT Arrow!")
                    return "RIGHT_ARROW"
                
                
    return None

while True:
    process_frame()  # Detect lines for movement
    detect_symbol()  # Detect circles, rectangles, arrows

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break

cv2.destroyAllWindows()
camera.stop()