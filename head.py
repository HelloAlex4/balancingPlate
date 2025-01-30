import movementFunctions as mv
import trackingFunctions as tr

xAngleAdjustor = 0
yAngleAdjustor = 0

pastXError = []
pastYError = []
pastVectorSpeed = []

def evaluateAdjustion(xError, yError, vectorSpeed):
    global pastXError
    global pastYError
    global pastVectorSpeed

    global xAngleAdjustor
    global yAngleAdjustor

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

    if averageVectorSpeed < 5:
        if abs(averageXError) > 90:
            xAngleAdjustor += (averageXError / 90) * -0.5
            
            pastXError = []
            pastYError = []
            pastVectorSpeed = []

        if abs(averageYError) > 90:
            yAngleAdjustor += (averageYError / 90) * -0.5

            pastXError = []
            pastYError = []
            pastVectorSpeed = []
        

def evaluateAngle(coords, xVelocity, yVelocity, vectorSpeed):
    evaluateAdjustion(coords[0] - goalX, coords[1] - goalY, vectorSpeed)

    xAngle = 0
    yAngle = 0

    xAngle = (xVelocity ** 1.2 if xVelocity >= 0 else -(-xVelocity) ** 1.2) / 90  * -2

    yAngle = (yVelocity ** 1.2 if yVelocity >= 0 else -(-yVelocity) ** 1.2) / 90 * -2

    value = (coords[0] - goalX) / 450
    xAngle += (value ** 1 if value >= 0 else -(-value) ** 1) * -3
    
    value = (coords[1] - goalY) / 450
    yAngle += (value ** 1 if value >= 0 else -(-value) ** 1) * -3

    return xAngle + xAngleAdjustor, yAngle + yAngleAdjustor

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

        angles = evaluateAngle(coords, Xvelocity, Yvelocity, vectorspeed)

        print(coords)
        print(Xvelocity)
        print(Yvelocity)
        print(vectorspeed)
        print(angles)

        mv.setPlateAngle(angles[0], angles[1])
    else:
        print("no object found")
