import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Try 1 or 2 if 0 doesnt work

if not cap.isOpened():
    print("? Error: Camera not detected! Check connection.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("? Failed to capture frame. Check camera.")
        break

    cv2.imshow("Camera Test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()