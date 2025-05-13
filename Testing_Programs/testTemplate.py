import cv2
import numpy as np
import time
from picamera2 import Picamera2

template_left = cv2.imread("left-arrow.jpg", 0)  # Load in grayscale

if template_left is None:
    print("Error: Template image not found!")
    exit()

print("Template shape:", template_left.shape)
