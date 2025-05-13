import cv2
import numpy as np
import time
from picamera2 import Picamera2

# Initialize PiCamera2
camera = Picamera2()
camera.preview_configuration.main.size = (320, 240)
camera.preview_configuration.main.format = "RGB888"
camera.preview_configuration.controls.FrameRate = 30
camera.configure("preview")
camera.start()
time.sleep(2)  # Allow camera to warm up

while True:
    # Capture frame from PiCamera2
    frame = camera.capture_array()
    
    greyImg = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) #1st Convert to grayscale 
    blurImg = cv2.GaussianBlur(greyImg, (5,5), 1.1) #2nd Apply Gaussian blur
    _, otsu_thresh = cv2.threshold(blurImg, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)#3rd Apply thresholding (inverted => whitebackground, darkobject) 
    edgeImg = cv2.Canny(otsu_thresh, 50, 150) #4th Apply Canny Edge Detection   
    
    contours, hierarchy = cv2.findContours(edgeImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        cnts = sorted(contours, key=cv2.contourArea, reverse = True) # Picks the contour
        
        if len(contours) > 1:
            for inner_cnt in cnts[1:]:  # Second largest contour (inside the square)
                cv2.drawContours(frame, [inner_cnt], 0, (0, 255, 0), 3)





    # Display the frame
    cv2.imshow("Pi Camera Feed", frame)
    cv2.imshow("Gaussian Blur", blurImg)
    cv2.imshow("Canny Edge", edgeImg) 
    cv2.imshow("Threshold", otsu_thresh)  # Show the binary image used for contours
 

    # Exit when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
camera.stop()  # Stop the camera properly
cv2.destroyAllWindows()
