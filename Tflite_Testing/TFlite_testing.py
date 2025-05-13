import sys
sys.path.insert(0,"/home/pi/tflite_env/lib/python3.11/site-packages")
import tflite_runtime.interpreter as tflite
import libcamera 
import cv2
from picamera2 import Picamera2
import numpy as np 

camera = Picamera2()
camera.preview_configuration.main.size = (320, 240)
camera.preview_configuration.main.format = "RGB888"
camera.preview_configuration.controls.FrameRate = 30
camera.configure("preview")
camera.start()

def load_labels(path):
    with open(path, "r") as f:
        return [line.strip() for line in f.readlines()]
    
def load_model(model_path):
    interpreter = tflite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    return interpreter

def preprocess_image(image, input_shape):
    image = cv2.resize(image, (input_shape[1], input_shape[2])) #Resize
    image = np.expand_dims(image, axis=0) #Add both dimension 
    image = image.astype(np.float32) / 255.0 #Normalize if needed
    return image

def classify_image(interpreter, image):
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
 
    image = preprocess_image(image, input_details[0]['shape'])
 
    # Set input tensor
    interpreter.set_tensor(input_details[0]['index'], image)
    interpreter.invoke()
 
    # Get output tensor
    output_data = interpreter.get_tensor(output_details[0]['index'])
    return output_data
 

model_path = "model_unquant.tflite"
labels_path = "labels.txt"
    
labels = load_labels(labels_path)
interpreter = load_model(model_path)
 
 

while True:
    frame = camera.capture_array()
    cv2.imshow("Pi Camera Feed", frame)
    
    
    predictions = classify_image(interpreter, frame)
    label_id = np.argmax(predictions)
    confidence = predictions[0][label_id]
    label_text = f"{labels[label_id]}: {confidence:.2f}"
 
    # Display result on frame
    cv2.putText(frame, label_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Real-time Classification", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
camera.stop()  # Stop the camera properly
cv2.destroyAllWindows() 