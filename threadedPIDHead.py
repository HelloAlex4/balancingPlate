import movementFunctions as mv
import trackingFunctions as tr
import threading
import time

class PIDController:
    def __init__(self, kp, ki, kd, buffer_size=5, max_rate=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.error_buffer = []
        self.buffer_size = buffer_size
        self.max_rate = max_rate
        self.last_output = 0

    def compute(self, setpoint, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Calculate current error
        error = setpoint - measured_value
        
        # Add to error buffer
        self.error_buffer.append(error)
        if len(self.error_buffer) > self.buffer_size:
            self.error_buffer.pop(0)
            
        # Calculate averaged error
        avg_error = sum(self.error_buffer) / len(self.error_buffer)
        
        # Only accumulate integral if error is large enough
        if abs(avg_error) > 90:
            self.integral += avg_error * dt
            
        derivative = (avg_error - self.prev_error) / dt if dt > 0 else 0
        
        output = (self.kp * avg_error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        # Limit rate of change
        output_change = output - self.last_output
        output_change = max(min(output_change, self.max_rate), -self.max_rate)
        output = self.last_output + output_change
        
        self.prev_error = avg_error
        self.last_time = current_time
        self.last_output = output
        
        return output

# Initialize PID controllers with smoothing
pid_x = PIDController(kp=0.01, ki=0.002, kd=0.007, buffer_size=5, max_rate=7.0)
pid_y = PIDController(kp=0.01, ki=0.002, kd=0.007, buffer_size=5, max_rate=7.0)

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