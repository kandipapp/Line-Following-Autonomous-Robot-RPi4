from picamera.array import PiRGBArray
from picamera2 import PiCamera2
import cv2
import numpy as np
import time

# Initialize PiCamera
camera = PiCamera2()
camera.resolution = (320, 240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320, 240))

# Allow camera to warm up
time.sleep(0.1)

# ROI parameters
roi_y_start = 180  # start of ROI (height)
roi_y_end = 240    # end of ROI (bottom of the frame)

print("Starting line detection...")

# Capture frames continuously
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    # Define ROI (bottom portion of the frame)
    roi = image[roi_y_start:roi_y_end, :]

    # Convert to grayscale
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # Gaussian blur to reduce noise
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Threshold to detect dark line on light background
    _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours and find center of the line
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])  # Center x of the line
            cy = int(M["m01"] / M["m00"])  # Center y of the line
            cv2.circle(roi, (cx, cy), 5, (0, 255, 0), -1)

            # Optional: Draw direction guide
            cv2.line(roi, (cx, 0), (cx, 60), (255, 0, 0), 2)

    # Show images
    cv2.imshow("Original Frame", image)
    cv2.imshow("ROI", roi)
    cv2.imshow("Threshold", thresh)

    key = cv2.waitKey(1) & 0xFF
    rawCapture.truncate(0)

    # Exit on 'q'
    if key == ord("q"):
        break

cv2.destroyAllWindows()