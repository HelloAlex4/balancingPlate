import time
import numpy as np
from scipy.optimize import fsolve
import gpiod
import math
import threading
from picamera2 import Picamera2
import cv2

# GPIO pin configurations for each motor
motors = [
    {"name": "Motor 1", "dir": 17, "step": 27, "ticks": 0},
    {"name": "Motor 2", "dir": 23, "step": 24, "ticks": 0},
    {"name": "Motor 3", "dir": 5, "step": 6, "ticks": 0}
]

currentYAngle = 0
currentXAngle = 0

# Initialize GPIO
chip = gpiod.Chip('gpiochip0')
motor_lines = {}

# Set GPIO 13, 16, and 26 to high permanently
gpio13_line = chip.get_line(13)
gpio13_line.request(consumer="motor_test", type=gpiod.LINE_REQ_DIR_OUT)
gpio13_line.set_value(1)

gpio16_line = chip.get_line(16)
gpio16_line.request(consumer="motor_test", type=gpiod.LINE_REQ_DIR_OUT)
gpio16_line.set_value(1)

gpio26_line = chip.get_line(26)
gpio26_line.request(consumer="motor_test", type=gpiod.LINE_REQ_DIR_OUT)
gpio26_line.set_value(1)

# Setup GPIO pins for all motors
for motor in motors:
    motor_lines[f"{motor['name']}_dir"] = chip.get_line(motor['dir'])
    motor_lines[f"{motor['name']}_step"] = chip.get_line(motor['step'])
    motor_lines[f"{motor['name']}_dir"].request(consumer="motor_test", type=gpiod.LINE_REQ_DIR_OUT)
    motor_lines[f"{motor['name']}_step"].request(consumer="motor_test", type=gpiod.LINE_REQ_DIR_OUT)

def calculate_height(theta):
    """
    Calculate the height of the endpoint of the system based on the given angle theta.

    Parameters:
        theta (float): Angle in degrees.

    Returns:
        float: Height of the endpoint in cm.
    """
    # Constants
    L1 = 5  # Length of the first arm in cm
    L2 = 9.68  # Length of the second arm in cm
    x2 = 0  # Fixed horizontal position of the endpoint in cm

    # Convert theta to radians
    theta_rad = math.radians(theta)

    # Calculate phi using the horizontal constraint
    cos_phi = (x2 - L1 * math.cos(theta_rad)) / L2
    if abs(cos_phi) > 1:
        raise ValueError("Geometry not possible for the given theta and system constraints.")
    
    phi_rad = math.acos(cos_phi)

    # Calculate total height
    h = L1 * math.sin(theta_rad) + L2 * math.sin(phi_rad)
    return h

def calculate_theta(h_desired):
    """
    Calculate the angle theta for the given desired height.

    Parameters:
        h_desired (float): The desired height in cm.

    Returns:
        float: The angle theta in degrees.
    """
    # Constants
    L1 = 5  # Length of the first arm in cm
    L2 = 9.68  # Length of the second arm in cm
    x2 = 0  # Fixed horizontal position of the endpoint in cm

    # Function to calculate the height
    def height_function(theta):
        theta_rad = np.radians(theta)  # Convert theta to radians
        acos_arg = (x2 - L1 * np.cos(theta_rad)) / L2
        # Ensure the argument of acos is within valid range [-1, 1]
        acos_arg = np.clip(acos_arg, -1, 1)
        return L1 * np.sin(theta_rad) + L2 * np.sin(np.arccos(acos_arg)) - h_desired

    # Use fsolve to find the solution for theta
    initial_guess = 30  # Initial guess in degrees
    theta_solution = fsolve(height_function, initial_guess, xtol=1e-8)

    return theta_solution[0]  # Return the first (and only) solution

def goToAngle(motor_index, angle, speed):
    currentAngle = motors[motor_index]['ticks'] * 0.9 - 30

    if currentAngle > angle:
        direction = 0
    else:
        direction = 1
    degree = abs(currentAngle - angle)
    rotate_motor(motors[motor_index], direction, degree, speed)  # Pass the motor dictionary instead of index

def goToHeight(motor_index, height, speed):
    desiredAngle = calculate_theta(height)
    goToAngle(motor_index, desiredAngle, speed)

def rotate_motor(motor, direction, degree, speed):
    speed = 0.002

    # Set direction (0 for clockwise, 1 for counterclockwise)
    motor_lines[f"{motor['name']}_dir"].set_value(direction)
    
    steps = int(degree * 400 / 360)  # Calculate steps based on degree
    
    # Find the motor index in the motors list
    motor_index = next(i for i, m in enumerate(motors) if m['name'] == motor['name'])
    
    if direction == 0:
        motors[motor_index]['ticks'] -= steps
    else:
        motors[motor_index]['ticks'] += steps

    # Perform steps
    for _ in range(steps):
        motor_lines[f"{motor['name']}_step"].set_value(1)
        time.sleep(speed)  # Adjust delay as needed
        motor_lines[f"{motor['name']}_step"].set_value(0)
        time.sleep(speed)  # Adjust delay as needed

def centerPlate():
    threads = [
        threading.Thread(target=goToHeight, args=(0, 10.5, 0.001)),
        threading.Thread(target=goToHeight, args=(1, 10.5, 0.001)),
        threading.Thread(target=goToHeight, args=(2, 10.5, 0.001))
    ]

    currentXAngle = 0
    currentYAngle = 0

    # Start all threads
    for thread in threads:
        thread.start()

    # Wait for all threads to complete
    for thread in threads:
        thread.join()

def resetPlate():
    threads = [
        threading.Thread(target=goToAngle, args=(0, -30, 0.001)),
        threading.Thread(target=goToAngle, args=(1, -30, 0.001)),
        threading.Thread(target=goToAngle, args=(2, -30, 0.001))
    ]

    # Start all threads
    for thread in threads:
        thread.start()

    # Wait for all threads to complete
    for thread in threads:
        thread.join()

def setPlateAngle(xAngle, yAngle):
    global motor1desiredHeight
    global motor2desiredHeight
    global motor3desiredHeight

    motor1desiredHeight = 10.5
    motor2desiredHeight = 10.5
    motor3desiredHeight = 10.5

    # Board and motor geometry
    boarddiameterShortWay = np.sin(np.deg2rad(30)) * 11
    centerDistance = np.cos(np.deg2rad(30)) * 11

    # Calculate height deltas for xAngle
    heightDeltaX1 = np.tan(np.deg2rad(xAngle)) * boarddiameterShortWay
    heightDeltaX2 = np.tan(np.deg2rad(xAngle)) * 11
    heightDeltaY = np.tan(np.deg2rad(yAngle)) * (centerDistance * 2)

    # Combine adjustments for xAngle and yAngle
    motor1desiredHeight += (heightDeltaX1) + (heightDeltaY / 2)
    motor2desiredHeight += (heightDeltaX1) - (heightDeltaY / 2)
    motor3desiredHeight -= (heightDeltaX2)

    # Move motors using threads
    threads = [
        threading.Thread(target=goToHeight, args=(0, motor1desiredHeight, 0.001)),
        threading.Thread(target=goToHeight, args=(1, motor2desiredHeight, 0.001)),
        threading.Thread(target=goToHeight, args=(2, motor3desiredHeight, 0.001))
    ]

    # Start all threads
    for thread in threads:
        thread.start()

    # Wait for all threads to complete
    for thread in threads:
        thread.join()


centerPlate()
time.sleep(2)

def oscillateMotor(motor_index, min_height, max_height, iterations, delay):
    """
    Oscillate a motor between a minimum and maximum height.

    Parameters:
        motor_index (int): Index of the motor to oscillate.
        min_height (float): Minimum height in cm.
        max_height (float): Maximum height in cm.
        iterations (int): Number of oscillation cycles.
        delay (float): Delay in seconds between each movement.
    """
    for _ in range(iterations):
        # Move to maximum height
        goToHeight(motor_index, max_height, 0.001)
        time.sleep(delay)
        # Move to minimum height
        goToHeight(motor_index, min_height, 0.001)
        time.sleep(delay)

def oscillateMotorsOutOfSync(min_height, max_height, iterations, delay, sync_delay):
    """
    Oscillate multiple motors out of sync.

    Parameters:
        min_height (float): Minimum height in cm for oscillation.
        max_height (float): Maximum height in cm for oscillation.
        iterations (int): Number of oscillation cycles.
        delay (float): Delay in seconds between each movement.
        sync_delay (float): Delay in seconds to start each motor out of sync.
    """
    threads = []
    for motor_index in range(len(motors)):
        # Create a thread for each motor with a delay to start out of sync
        thread = threading.Thread(target=oscillateMotor, args=(motor_index, min_height, max_height, iterations, delay))
        threads.append(thread)
        thread.start()
        time.sleep(sync_delay)  # Delay to start the next motor out of sync

    # Wait for all threads to complete
    for thread in threads:
        thread.join()


#ball balancer
def initialize_camera():
    # Initialize Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (1000, 1000)})  # Adjust resolution
    picam2.configure(config)
    picam2.start()
    return picam2

def find_orange_object_coordinates(picam2):
    # Define the HSV range for orange color
    lower_orange = np.array([15, 150, 150])  # Narrowed range for hue, saturation, and value
    upper_orange = np.array([20, 255, 255])  # Narrowed range for hue, saturation, and value

    # Capture a single frame
    frame = picam2.capture_array()

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
            coordinates = (center_x, center_y)

    return coordinates

previousCoords = []

def getBallVelocityVector(coords):
    previousCoords.append(coords)
    if len(previousCoords) < 6:
        return (0, 0)
    
    previousX, previousY = previousCoords.pop(0)

    XDelta = 0
    YDelta = 0
    for x in previousCoords:
        XDelta += x[0] - previousX
        YDelta += x[1] - previousY

        previousX = x[0]
        previousY = x[1]

    averageXDelta = XDelta / 5
    averageYDelta = YDelta / 5

    speed = math.sqrt(averageXDelta**2 + averageYDelta**2)

    return averageXDelta, averageYDelta, speed

def evaluateAngle(coords, xGoal, yGoal, XspeedDelta, YspeedDelta):
    xAngle = 0
    yAngle = 0

    xAngle = XspeedDelta / 30 * -5

    yAngle = YspeedDelta / 30 * -5


    value = (coords[0] - xGoal) / 450
    xAngle += (value ** 0.8 if value >= 0 else -(-value) ** 0.8) * -4
    
    value = (coords[1] - yGoal) / 450
    yAngle += (value ** 0.8 if value >= 0 else -(-value) ** 0.8) * -4

    return yAngle, xAngle


# Initialize the camera once
camera = initialize_camera()

fool = input("lay the ball on top")

tick = 0

goalX = 600
goalY = 600
#kjf
while True:
    tick+=1

    coords = find_orange_object_coordinates(camera)
    if coords:
        velocityData = getBallVelocityVector(coords)

        XspeedDelta = velocityData[0]
        YspeedDelta = velocityData[1]
        vectorSpeed = math.sqrt(XspeedDelta**2 + YspeedDelta**2)

        print(vectorSpeed)

        angles = evaluateAngle(coords, goalX, goalY, XspeedDelta, YspeedDelta)

        setPlateAngle(angles[0], angles[1])


        print(f"Largest Orange Object - X: {coords[0]}, Y: {coords[1]}")
        print(f"XAngle:  {angles[0]}  YAngle:  {angles[1]}")
    else:
        print("No orange object detected.")

    goalX = 600
    goalY = 600


        

# Stop the camera when done
camera.stop()
cv2.destroyAllWindows()

resetPlate()