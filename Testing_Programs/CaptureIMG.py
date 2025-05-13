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
    
    cv2.imshow("Camera Feed - Press 'C' to capture", frame)
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur_frame = cv2.GaussianBlur(gray_frame, (5,5), 1.1)
    _, otsu_frame = cv2.threshold(blur_frame, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    cv2.imshow("BW", otsu_frame)
    
    key = cv2.wait2Key(1) & 0xFF
    
    if key == ord('c'):
        # Save the captured image
        cv2.imwrite(".jpg", frame)
        print("Image Captured and Saved as 'captured.jpg'")

    elif key == ord('q'):
        # Exit the loop and close window
        break
    
cv2.destroyAllWindows()
camera.stop()   
