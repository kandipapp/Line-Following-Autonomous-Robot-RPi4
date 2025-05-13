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
    
    # Display the frame
    cv2.imshow("Pi Camera Feed", frame)
    # Exit when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
camera.stop()  # Stop the camera properly
cv2.destroyAllWindows() 