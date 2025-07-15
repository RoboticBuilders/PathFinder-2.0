from utils import *
# from simulator_utils import *

def homeToLineTopMiddle():
    # Starts at the normal full run start position, at 180 degrees

    # Ends so that the color sensor has caught the line topMiddle and is pointing 180 deg

    angle = 180
    gyroStraightWithDrive(distance=12,speed=800,targetAngle=angle, backward=True)
    drive_base.settings(800, 800, 800, 800)
    curve(radius=-160,angle = 90)

    angle = 90
    gyroStraightWithDrive(distance=17,speed=500,targetAngle=angle,tillBlackLine=True, backward=True)
    gyroStraightWithDrive(distance=31,speed=500,targetAngle=angle, backward=True)
    curve(radius=-200,angle = -90)

    angle = 180
    gyroStraightWithDrive(distance=35,speed=500,targetAngle=angle,tillBlackLine=True, backward=True)

def homeToLeftGreySquare():
    # Starts at the normal full run start position, at 180 degrees

    # Ends pointing the left grey square and is centered at the 
    # block and color sensor beam is 6 cm from the light grey edge

    angle = 180
    gyroStraightWithDrive(distance=12,speed=500,targetAngle=angle, backward=True)
    drive_base.settings(800, 800, 800, 800)
    curve(radius=-160,angle = 90)

    angle = 90
    gyroStraightWithDrive(distance=17,speed=400,targetAngle=angle,tillBlackLine=True, backward=True, slowDown=False)
    gyroStraightWithDrive(distance=42,speed=500,targetAngle=angle, backward=True)

    angle = 0
    turnToAngle(targetAngle=angle, speed = 300)
    gyroStraightWithDrive(distance=15,speed=300,targetAngle=angle)

def homeToDrone():
    # ** This is assuming that the drone is not there at its start area
    # Starts at 0 deg and the inside right wheel is against 
    # the yellow line and back is at the back arm

    # Ends 4 cm away from the drone start area square

    angle = 0
    gyroStraightWithDrive(distance=45, speed=400, targetAngle=angle, slowDown=False)
    gyroStraightWithDrive(distance=15, speed=400, targetAngle= 10, slowDown=False)
    gyroStraightWithDrive(distance=18, speed=400, targetAngle=angle)

def homeToRightGreySquare():
     # ** This is assuming that the drone is not there at its start area
    # Starts at 0 deg and the inside right wheel is against 
    # the yellow line and back is at the back arm

    # Ends 4 cm away from the drone start area square

    angle = 0
    gyroStraightWithDrive(distance=45, speed=400, targetAngle=angle, slowDown=False)
    gyroStraightWithDrive(distance=22, speed=400, targetAngle= 18, slowDown=False)
    gyroStraightWithDrive(distance=38, speed=400, targetAngle=angle)

def homeToAramco():
     # ** This is assuming that the drone is not there at its start area
    # Starts at 0 deg and the inside right wheel is against 
    # the yellow line and back is at the back arm

    # Ends 4 cm away from the drone start area square

    angle = 0
    gyroStraightWithDrive(distance=45, speed=400, targetAngle=angle, slowDown=False)
    gyroStraightWithDrive(distance=28, speed=400, targetAngle= -23, slowDown=False)
    gyroStraightWithDrive(distance=38, speed=400, targetAngle=angle)

def middleTopLineToAramco():
    curve(radius = -300, angle = -90, speed = 700, stop = Stop.COAST)
    gyroStraightWithDrive(distance = 20, speed = 500, backward = True, targetAngle = -90, tillBlackLine = True, stop = Stop.COAST)
    gyroStraightWithDrive(distance = 30, speed = 700, backward = True, targetAngle = -90)
    
    angle = 0
    turnToAngle(targetAngle = angle, speed = 700)
    drive_base.straight(50)

def middleTopLineToRightGreySquare():
    curve(radius = -300, angle = -90, speed = 700, stop = Stop.COAST)
    gyroStraightWithDrive(distance = 20, speed = 500, backward = True, targetAngle = -90, tillBlackLine = True, stop = Stop.COAST)
    gyroStraightWithDrive(distance = 30, speed = 700, backward = True, targetAngle = -90)
    
    angle = 90
    turnToAngle(targetAngle = angle, speed = 700)

def middleTopLineToTopLineDropoffs():
    gyroStraightWithDrive(distance = 66, speed = 700, backward = True, targetAngle = -173)
    
    angle = 0
    turnToAngle(targetAngle = angle)

def middleTopLineToBottomLineDropoffs():
    curve(radius = -300, angle = -90, speed = 700, stop = Stop.COAST)
    gyroStraightWithDrive(distance = 20, speed = 500, backward = True, targetAngle = -90, tillBlackLine = True, stop = Stop.COAST)
    gyroStraightWithDrive(distance = 40, speed = 700, backward = True, targetAngle = -90)
    
    angle = 0
    turnToAngle(targetAngle = angle, speed = 700)
    gyroStraightWithDrive(distance = 30, speed = 700, targetAngle = angle)

def middleTopLineToLineInFrontOfBlocks():
    curve(radius = -300, angle = -90, speed = 700, stop = Stop.COAST)
    gyroStraightWithDrive(distance = 20, speed = 500, backward = True, targetAngle = -90, tillBlackLine = True)
    turnToAngle(targetAngle = 180, speed = 500)

def middleTopLineToPark():
    curve(radius = -300, angle = -90, speed = 700, stop = Stop.COAST)
    gyroStraightWithDrive(distance = 20, speed = 500, backward = True, targetAngle = -90, tillBlackLine = True)

    angle = 0
    turnToAngle(targetAngle = angle, speed = 500)
    gyroStraightWithDrive(distance = 20, speed = 700, targetAngle = angle)

def aramcoToMiddleTopLine():
    # Inverse of middleTopLineToAramco: go from Aramco to the middle top line
    drive_base.straight(-50)
    angle = -90
    turnToAngle(targetAngle=angle, speed=700)
    gyroStraightWithDrive(distance=30, speed=700, backward=False, targetAngle=angle)
    gyroStraightWithDrive(distance=20, speed=500, backward=False, targetAngle=angle, tillBlackLine=True, stop=Stop.COAST)
    gyroStraightWithDrive(distance = 10, speed = 700, targetAngle = -90)
    curve(radius=300, angle=-90, speed=700, stop=Stop.COAST)

#initializeAndWaitForRobotReady()
#hub.imu.reset_heading(180)
#homeToLineTopMiddle()