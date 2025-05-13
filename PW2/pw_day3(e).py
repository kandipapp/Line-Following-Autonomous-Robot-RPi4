import cv2
import numpy as np
import time
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
        
# for name, img in processed_symbols.items():
#     cv2.imshow(name, img)  # Show the processed image
#     cv2.waitKey(1000)  # Wait for 1 second (1000 ms)
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
    frame = camera.capture_array()
    
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur_frame = cv2.GaussianBlur(gray_frame, (5,5), 1.1)
    _, otsu_frame = cv2.threshold(blur_frame, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    cv2.imshow('Symbol', otsu_frame)

    best_match = None
    max_matches = 0

    contours, hierarchy = cv2.findContours(otsu_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    detected_shape = None
    
    if contours:
        cnts = sorted(contours, key=cv2.contourArea, reverse = True) # Picks the contour
        if cnts:
            if len(contours) <2:
                cv2.drawContours(frame, [cnts[0]], -1, (0, 255, 0), 3)
                detected_shape = detect_shape(cnts[0])
                cv2.putText(frame, f"{detected_shape}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, (0, 255, 0), 2, cv2.LINE_AA)

            else:
                for name, ref_img in processed_symbols.items():
                    matches = detect_with_orb(ref_img, otsu_frame)  

                    if matches > max_matches:
                        max_matches = matches
                        best_match = name

                if best_match:
                    cv2.putText(frame, f"{best_match}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                                1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow('Symbol Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


camera.stop()
cv2.destroyAllWindows()