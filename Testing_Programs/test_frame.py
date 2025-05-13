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

while True: 
    frame = camera.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, 50, 150)  # Apply edge detection
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        cnts = sorted(contours, key=cv2.contourArea, reverse = True) # Picks the contour
        
        if len(contours) > 1:
            for inner_cnt in cnts[1:]:  # Second largest contour (inside the square)
                cv2.drawContours(frame, [inner_cnt], 0, (0, 255, 0), 3)



    # Display the frame
    cv2.imshow("Pi Camera Feed", frame)
    cv2.imshow("Gray", gray)
    cv2.imshow("Edges", edges) 


        # Exit when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
camera.stop()  # Stop the camera properly
cv2.destroyAllWindows() 