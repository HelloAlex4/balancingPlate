import movementFunctions as mv
import trackingFunctions as tr

xAngleAdjustor = 0
yAngleAdjustor = 0

pastXError = []
pastYError = []
pastVectorSpeed = []

adjustorCountdown = 0

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.integral_x = 0
        self.integral_y = 0
        
    def compute(self, error_x, error_y):
        # Proportional term
        p_x = self.kp * error_x
        p_y = self.kp * error_y
        
        # Integral term
        self.integral_x += error_x
        self.integral_y += error_y
        i_x = self.ki * self.integral_x
        i_y = self.ki * self.integral_y
        
        # Derivative term
        d_x = self.kd * (error_x - self.prev_error_x)
        d_y = self.kd * (error_y - self.prev_error_y)
        
        # Update previous error
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        
        # PID output
        output_x = p_x + i_x + d_x
        output_y = p_y + i_y + d_y
        
        return output_x, output_y

pid_controller = PIDController(kp=0.008, ki=0.0002, kd=0.07)

def evaluateAngle(coords):
    error_x = coords[0] - goalX
    error_y = coords[1] - goalY
    
    # Get PID adjustments
    xAngle, yAngle = pid_controller.compute(error_x, error_y)
    
    return -xAngle, -yAngle

goalX = 600
goalY = 600

tick = 0

mv.centerPlate()

wait = input("put the ball on the plate")

while True:
    tick += 1

    coords = tr.find_orange_object_coordinates()
    if coords:
        velocityData = tr.getBallVelocityVector(coords)
        Xvelocity = velocityData[0]
        Yvelocity = velocityData[1]
        vectorspeed = velocityData[2]

        if velocityData == (0, 0, 0):
            continue

        angles = evaluateAngle(coords)

        print(coords)
        print(Xvelocity)
        print(Yvelocity)
        print(vectorspeed)
        print(angles)

        mv.setPlateAngle(angles[0], angles[1])
    else:
        print("no object found")
