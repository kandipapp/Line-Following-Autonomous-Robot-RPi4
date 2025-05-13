import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize Pi Camera
camera = Picamera2()
camera.preview_configuration.main.size = (320, 240)
camera.preview_configuration.main.format = "RGB888"
camera.configure("preview")
camera.start()

# HSV range you want to test (adjust as needed)
# Example: red
lower_hsv = np.array([50, 10, 10])
upper_hsv = np.array([180, 255, 255])

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        pixel = frame[y, x]
        hsv_pixel = hsv[y, x]

        print(f"RGB: {pixel} | HSV: {hsv_pixel}")

cv2.namedWindow("Live Feed")
cv2.setMouseCallback("Live Feed", mouse_callback)

print("Hover mouse over a color to see RGB and HSV values")

while True:
    frame = camera.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Mask and count pixels within HSV range
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    pixel_count = cv2.countNonZero(mask)

    # Show the count on the image
    cv2.putText(frame, f"Matched Pixels: {pixel_count}", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Show windows
    cv2.imshow("Live Feed", frame)
    cv2.imshow("Mask", mask)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
camera.stop()
