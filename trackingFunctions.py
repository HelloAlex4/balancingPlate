import time
import numpy as np
from scipy.optimize import fsolve
import gpiod
import math
import threading
from picamera2 import Picamera2
import cv2

def initialize_camera():
    # Initialize Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (1000, 1000)})  # Adjust resolution
    picam2.configure(config)
    picam2.start()
    return picam2

def find_orange_object_coordinates():
    # Define the HSV range for orange color
    lower_orange = np.array([15, 150, 150])  # Narrowed range for hue, saturation, and value
    upper_orange = np.array([20, 255, 255])  # Narrowed range for hue, saturation, and value

    # Capture a single frame
    frame = camera.capture_array()

    # Convert from RGB to BGR for OpenCV
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Convert the frame to HSV
    hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # Create a mask for orange color
    mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)

    # Find contours of the orange regions
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    coordinates = None

    # If there are contours, find the largest one
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if area > 500:  # Ignore small clusters
            # Get the bounding box coordinates
            x, y, w, h = cv2.boundingRect(largest_contour)
            # Calculate the center coordinates
            center_x = x + w // 2
            center_y = y + h // 2
            coordinates = (center_y, center_x)

    return coordinates

previousCoords = []

def getBallVelocityVector(coords):
    previousCoords.append(coords)
    if len(previousCoords) < 4:
        return (0, 0, 0)
    
    previousX, previousY = previousCoords.pop(0)

    XDelta = 0
    YDelta = 0
    for x in previousCoords:
        XDelta += x[0] - previousX
        YDelta += x[1] - previousY

        previousX = x[0]
        previousY = x[1]

    averageXDelta = XDelta / 3
    averageYDelta = YDelta / 3

    speed = math.sqrt(averageXDelta**2 + averageYDelta**2)

    return averageXDelta, averageYDelta, speed


camera = initialize_camera()