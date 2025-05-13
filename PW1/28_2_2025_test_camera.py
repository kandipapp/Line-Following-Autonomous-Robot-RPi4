import cv2

# Define GStreamer pipeline
gst_pipeline = (
    "libcamerasrc ! video/x-raw,format=NV12,width=1280,height=720,framerate=30/1 ! "
    "videoconvert ! video/x-raw,format=BGR ! appsink"
)

# Open video capture using GStreamer pipeline
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: Couldn't open camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Couldn't read frame.")
        break
    
    # Display the frame
    cv2.imshow("Raspberry Pi Camera", frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()