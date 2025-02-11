import movementFunctions as mv
import trackingFunctions as tr
import threading
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, setpoint, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time
        
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        self.prev_error = error
        self.last_time = current_time
        
        return output

# Initialize PID controllers for X and Y axes
kpValue = 0.01
kiValue = 0.001
kdValue = 0.005
pid_x = PIDController(kp=kpValue, ki=kiValue, kd=kdValue)
pid_y = PIDController(kp=kpValue, ki=kiValue, kd=kdValue)

goalX = 600
goalY = 600
coords = None
angles = None

def evaluateAngle(coords):
    if not coords:
        return 0, 0
        
    # Compute PID outputs for both axes
    x_angle = pid_x.compute(goalX, coords[0])
    y_angle = pid_y.compute(goalY, coords[1])
    
    # Limit angles to safe values
    x_angle = max(min(x_angle, 15), -15)
    y_angle = max(min(y_angle, 15), -15)
    
    return x_angle, y_angle

def angleCalcThread():
    global angles
    global coords
    
    while True:
        coords = tr.find_orange_object_coordinates()
        if coords:
            velocityData = tr.getBallVelocityVector(coords)
            Xvelocity = velocityData[0]
            Yvelocity = velocityData[1]
            vectorspeed = velocityData[2]
            if velocityData == (0, 0, 0):
                continue
            angles = evaluateAngle(coords)

def movePlateThread():
    global angles
    while True:
        if angles:
            mv.setPlateAngle(angles[0], angles[1])

mv.centerPlate()

wait = input("put the ball on the plate")

threads = [
    threading.Thread(target=angleCalcThread),
    threading.Thread(target=movePlateThread)
]

for thread in threads:
    thread.start()

tick = 0

goalX = 600
goalY = 600

while True:
    time.sleep(0.01)
    print(angles)
    print(coords)
    tick += 1