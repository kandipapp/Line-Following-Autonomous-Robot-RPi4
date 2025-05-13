import cv2

import numpy as np

import time

from picamera2 import Picamera2



# Initialize PiCamera2

camera = Picamera2()
camera.preview_configuration.main.size = (320, 240)
camera.preview_configuration.main.format = "RGB888"
camera.preview_configuration.controls.FrameRate = 15
camera.configure("preview")
camera.start()
time.sleep(2)  # Allow camera to warm up



while True:
    # Capture frame from PiCamera2
    frame = camera.capture_array()

    # Convert to grayscale for better processing
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    
    # Thresholding to detect the black line
    lower_black = np.array([0, 0, 0], dtype="uint8")
    upper_black = np.array([60, 60, 60], dtype="uint8")
    blackline = cv2.inRange(frame, lower_black, upper_black)

    # Apply morphological transformations to clean the image
    kernel = np.ones((3, 3), np.uint8)
    blackline = cv2.erode(blackline, kernel, iterations=5)
    blackline = cv2.dilate(blackline, kernel, iterations=9)

    # Find contours of the black line
    contours_blk, hierarchy_blk = cv2.findContours(
        blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours_blk) > 0:
        # Get the largest contour (assuming it's the black line)
        blackbox = cv2.minAreaRect(contours_blk[0])
        (x_min, y_min), (w_min, h_min), angle = blackbox

        # Adjust angle values
        if angle < -45:
            angle = 90 + angle

        if w_min < h_min and angle > 0:
            angle = (90 - angle) * -1
            
        if w_min > h_min and angle < 0:
            angle = 90 + angle
        
        angle = int(angle)

        # Draw the contour and angle on the frame
        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 3)
        cv2.putText(
            frame,
            f"Angle: {angle}",
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2,
        )

    # Show the processed frame
    cv2.imshow("Original with Line", frame)

    # Exit when 'q' is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# Cleanup
cv2.destroyAllWindows()