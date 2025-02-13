import movementFunctions as mv
import trackingFunctions as tr
import threading
import time

xAngleAdjustor = 0
yAngleAdjustor = 0

pastXError = []
pastYError = []
pastVectorSpeed = []

adjustorCountdown = 0

goalX = 600
goalY = 600

coords = None

angles = None

tick = 0

def evaluateAdjustion(xError, yError, vectorSpeed):
    global pastXError
    global pastYError
    global pastVectorSpeed

    global xAngleAdjustor
    global yAngleAdjustor

    global adjustorCountdown

    if adjustorCountdown > 0:
        adjustorCountdown -= 1
        return

    pastXError.append(xError)
    pastYError.append(yError)
    pastVectorSpeed.append(vectorSpeed)

    averageXError = 0
    averageYError = 0
    averageVectorSpeed = 0

    if len(pastXError) > 6:
        for y in pastYError:
            averageXError += y

        for x in pastXError:
            averageYError += x

        for z in pastVectorSpeed:
            averageVectorSpeed += z
        
        averageYError = averageYError / 5
        averageXError = averageXError / 5
        averageVectorSpeed = averageVectorSpeed / 5

        pastXError.pop(0)
        pastYError.pop(0)
        pastVectorSpeed.pop(0)

    if adjustorCountdown < 10:
        if averageVectorSpeed < 2.5:
            if abs(averageXError) > 100:
                if averageXError > 0:
                    yAngleAdjustor -= 0.45
                else:
                    yAngleAdjustor += 0.45
                
                adjustorCountdown = 10
                
                pastXError = []
                pastYError = []
                pastVectorSpeed = []

            if abs(averageYError) > 100:
                if averageYError > 0:
                    xAngleAdjustor -= 0.45
                else:
                    xAngleAdjustor += 0.45

                adjustorCountdown = 10

                pastXError = []
                pastYError = []
                pastVectorSpeed = []
        

def evaluateAngle(coords, xVelocity, yVelocity, vectorSpeed):
    evaluateAdjustion(coords[0] - goalX, coords[1] - goalY, vectorSpeed)

    xAngle = 0
    yAngle = 0

    xAngle = (xVelocity ** 1 if xVelocity >= 0 else -(-xVelocity) ** 1) / 100  * -9.5

    yAngle = (yVelocity ** 1 if yVelocity >= 0 else -(-yVelocity) ** 1) / 100 * -9.5

    value = (coords[0] - goalX) / 450
    xAngle += (value ** 0.7 if value >= 0 else -(-value) ** 0.7) * -3
    
    value = (coords[1] - goalY) / 450
    yAngle += (value ** 0.7 if value >= 0 else -(-value) ** 0.7) * -3

    return xAngle + xAngleAdjustor, yAngle + yAngleAdjustor


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
            angles = evaluateAngle(coords, Xvelocity, Yvelocity, vectorspeed)

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

while True:
    time.sleep(0.01)
    print(angles)
    print(coords)
    tick += 1

    if tick > 100:
        goalX = 750
        goalY = 750
    if tick > 200:
        goalX = 350
        goalY = 750
    if tick > 300:
        goalX = 300
        goalY = 350
    if tick > 400:
        goalX = 750
        goalY = 350
    if tick > 500:
        goalX = 750
        goalY = 750
        tick = 10