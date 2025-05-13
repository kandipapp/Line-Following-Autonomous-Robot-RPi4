import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize Pi Camera
camera = Picamera2()
camera.preview_configuration.main.size = (320, 240)
camera.preview_configuration.main.format = "RGB888"
camera.configure("preview")
camera.start()

# Mouse callback to get RGB at clicked position
def get_pixel_color(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        r, g, b = frame[y, x]  # Since the frame is in RGB format
        print(f"Clicked at ({x}, {y}) ? RGB: ({r}, {g}, {b})")
        # Optional: draw info on image
        cv2.circle(display_frame, (x, y), 5, (255, 255, 255), -1)
        text = f"RGB: ({r},{g},{b})"
        cv2.putText(display_frame, text, (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

# Setup window
cv2.namedWindow("RGB Picker")
cv2.setMouseCallback("RGB Picker", get_pixel_color)

while True:
    frame = camera.capture_array()  # This is in RGB format
    display_frame = frame.copy()    # For drawing without altering the original

    cv2.imshow("RGB Picker", display_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
camera.stop()