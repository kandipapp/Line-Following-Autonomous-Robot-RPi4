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

def detect_symbol():
    frame = camera.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)  # Convert to grayscale
    _, mask = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)  # Normal binary thresholding
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        approx = cv2.approxPolyDP(contour,0.02*cv2.arcLength(contour,True),True)

        # Detect rectangle
        if len(approx) == 4:
            print("Rectangle detected!")

            # Get bounding box coordinates
            x, y, w, h = cv2.boundingRect(approx)

            # Crop the region inside the rectangle
            roi = gray[y:y+h, x:x+w]

            # Pass the cropped region to detect specific symbols
            detected_symbol = detect_specSymbol(roi)
            
            if detected_symbol:
                print(f"Detected Symbol: {detected_symbol}")

def detect_specSymbol(roi):
    """ Performs template matching inside a detected rectangular ROI """
    
    # Load and resize templates
    templates = {
        "LEFT_ARROW": "left-arrow.jpg",
        "RIGHT_ARROW": "right-arrow.jpg",
        "UP_ARROW": "up-arrow.jpg",
        "DOWN_ARROW": "down-arrow.jpg",
        "SEMIQUARDCIR": "semiquad-Cir.jpg"
    }

    best_match = None
    best_score = 0.5  # Minimum confidence threshold

    for symbol, path in templates.items():
        template = cv2.imread(path, 0)

        if template is not None:
            template = cv2.resize(template, (roi.shape[1]//2, roi.shape[0]//2))  # Resize to fit inside ROI
            res = cv2.matchTemplate(roi, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(res)

            if max_val > best_score:  # Check confidence score
                best_match = symbol
                best_score = max_val

    return best_match

while True:
    process_frame()  # Process each frame
    detect_symbol()  # Detect rectangles and symbols inside them

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break

cv2.destroyAllWindows()
camera.stop()