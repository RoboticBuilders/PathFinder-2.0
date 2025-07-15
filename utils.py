from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from umath import *
from pybricks import version
import urandom

hub = PrimeHub()    
sw = StopWatch()

left_motor = Motor(Port.E,Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A)
pickers = Motor(Port.C)
ball_picker = Motor(Port.F)
#Motor(Port.E)

color_sensor = ColorSensor(Port.D)
block_color = ColorSensor(Port.B)

# Set by testing, the actual diameter is 65
wheel_diameter = 62
# ----- Was 129-------
axle_track = 135
drive_base = DriveBase(left_motor, right_motor, wheel_diameter=wheel_diameter, axle_track=axle_track)
drive_base.use_gyro(True)
pickers_angle = 0

LOGToFILE = 1
# Color defaults
BLACK_COLOR = 30
WHITE_COLOR = 60

wheel_radius = wheel_diameter / 2
_CM_PER_INCH = 2.54
WHEEL_RADIUS_CM = wheel_radius / 10

# Constants for Force turn.
FORCETURN_RIGHT = 0
FORCETURN_LEFT = 1
FORCETURN_NONE = 2

# Line follower constants.
LINE_FOLLOWER_EDGE_LEFT = 0
LINE_FOLLOWER_EDGE_RIGHT = 1

def initializeAndWaitForRobotReady(waitForButtonPush=True):
    while True:
        if (hub.imu.ready() == True):
            break
    drive_base.reset()
    print("voltage: " + str(hub.battery.voltage()))
    
    # Setup the drivebase settings.
    setDriveBaseSettings(straight_speed=400,straight_acceleration=800,
                        turn_rate=400,turn_acceleration=300)

    hub.speaker.beep()
    drive_base.stop()
    if (waitForButtonPush == True):
        waitForRightButtonPress()

    # This is intentional because we want zero to be the length of the mat.
    resetPickers()
    resetBallPicker()
    resetHeading(0)
    
def resetHeading(heading):
    drive_base.stop()
    hub.imu.reset_heading(heading)

def waitForLeftButtonPress():
    # Wait for any button to be pressed, and save the result.
    sw.pause()
    pressed = []
    while not any(pressed):
        pressed = hub.buttons.pressed()
        if (Button.LEFT in pressed):
            sw.resume()
            return

def waitForRightButtonPress():
    # Wait for any button to be pressed, and save the result.
    sw.pause()
    pressed = []
    while not any(pressed):
        pressed = hub.buttons.pressed()
        if (Button.RIGHT in pressed):
            sw.resume()
            return

def runWithTimingAndStartBlockParam(function, name, startBlockParam):
    #sw = StopWatch()
    sw.resume()
    startTime = sw.time()
    retValue = function(startBlockParam)
    endTime = sw.time()
    totaltime = endTime-startTime
    print("RunTiming: " + name + ":" + str(totaltime))
    sw.pause()
    return retValue

def runWithTiming(function,name):
    global sw
    #sw = StopWatch()
    sw.resume()
    startTime = sw.time()
    retValue = function()
    endTime = sw.time()
    totaltime = endTime-startTime
    print("RunTiming: " + name + ":" + str(totaltime))
    
    sw.pause()
    return retValue

def getAverageLoad():
    left_load = abs(left_motor.load())
    right_load = abs(right_motor.load())
    return int((left_load + right_load) / 2)

def getAverageAngle():
    return (left_motor.angle() + right_motor.angle()) / 2

def convertDegToCM(degrees):
    return degrees * WHEEL_RADIUS_CM * pi * 2 / 360

def converCMToDeg(distance):
    return distance * 360 / (WHEEL_RADIUS_CM * pi * 2)

def convertInchesToCM(distanceInInches):
    """
    Convert Inches To CM
    ____________________
    """
    return _CM_PER_INCH * distanceInInches

def getDriveBase():
    return drive_base

def stopDriveBase(stop=Stop.BRAKE):
    drive_base.straight(distance=0,then=stop)


def getHeadingValue():
    return hub.imu.heading()

def getHeadingValueStr():
    return str(hub.imu.heading())

def setDriveBaseSettings(straight_speed, straight_acceleration, turn_rate, turn_acceleration):
    drive_base.settings(straight_speed, straight_acceleration, turn_rate, turn_acceleration)

def getDriveBaseSettings():
    return drive_base.settings()

def getReflectedLight():
    return color_sensor.reflection()

def getHSV():
    return color_sensor.hsv()

def getValue():
    return color_sensor.hsv().v

def getSaturation():
    return color_sensor.hsv().s

def _getValueInternal(color_sensor):
        return color_sensor.hsv().v



def getBlockReflectedLight():
    return block_color.reflection()

def getBlockHSV():
    return block_color.hsv()

def getBlockSaturation():
    return block_color.hsv().s

def getBlockHue():
    return block_color.hsv().h  

def getBlockValue():
    return block_color.hsv().v



def printDriveBaseValues():
    settings = drive_base.settings()
    if settings is not None:
        straight_speed, straight_acceleration, turn_rate, turn_acceleration = settings
        print("straight_speed: " + str(straight_speed))
        print("straight_acceleration: " + str(straight_acceleration))
        print("turn_rate: " + str(turn_rate))
        print("turn_acceleration: " + str(turn_acceleration))
    else:
        print("Drive base settings returned None")


# Convert angle to zero to 359 space.
# negative angles are also converted into zero to 359 space.
def _convertAngleTo360(angle):
    negative = False
    if angle < 0:
        angle = abs(angle)
        negative = True

    degreesLessThan360 = angle
    if angle >= 360:
        degreesLessThan360 = angle % 360
    
    if negative == True:
        degreesLessThan360 = 360 - degreesLessThan360

    return degreesLessThan360

# Use this for curving, as it sets the settings.
def curve(radius, angle, speed = 300, acceleration = 450, deceleration = 0, 
          optional_simulator_end_point = None, optional_simulator_end_angle = None,stop=Stop.HOLD):
    if deceleration == 0:
        deceleration = acceleration

    prev_straight_speed, prev_straight_acceleration, prev_turn_rate, prev_turn_acceleration=drive_base.settings()
    drive_base.settings(straight_speed=speed, straight_acceleration=(acceleration, deceleration), 
                        turn_rate=speed, turn_acceleration=(acceleration, deceleration))
    drive_base.curve(radius=radius, angle = angle,then=stop) 
    drive_base.settings(prev_straight_speed, prev_straight_acceleration, prev_turn_rate, prev_turn_acceleration)
    drive_base.stop()

# Use this for curving, as it sets the settings.
def curve_with_load_detect_to_get_out_park_b(radius, angle, speed = 300, acceleration = 450, deceleration = 0, 
          optional_simulator_end_point = None, optional_simulator_end_angle = None):
    if deceleration == 0:
        deceleration = acceleration

    prev_straight_speed, prev_straight_acceleration, prev_turn_rate, prev_turn_acceleration=drive_base.settings()
    drive_base.settings(straight_speed=speed, straight_acceleration=(acceleration, deceleration), 
                        turn_rate=speed, turn_acceleration=(acceleration, deceleration))
    drive_base.curve(radius=radius, angle = angle, wait=False)

    # Now calculate the load on the motors.
    # If we get two readings more than 90 we break out.
    second_reading_more_than_90 = False
    while(drive_base.done() == False):
        left_load = abs(left_motor.load())
        right_load = abs(right_motor.load())
        currLoad = int((left_load + right_load) / 2)
        
        if (currLoad > 90):
            if (second_reading_more_than_90 == False):
                second_reading_more_than_90 = True
            else:
                hub.speaker.beep()
                drive_base.stop()
                
                # backoff 30mm. Then reduce the curve and try the remaining part of the curve again.
                drive_base.straight(30)
                drive_base.curve(radius= -100, angle = -25)
                turnToAngle(targetAngle=0, speed=400)
                break
        
    drive_base.settings(prev_straight_speed, prev_straight_acceleration, prev_turn_rate, prev_turn_acceleration)

def turnToAngle(targetAngle, speed=400, turn_acceleration=400, turn_deceleration=450, 
                right_correction=-0.1, left_correction = -0.1, forceTurn = FORCETURN_NONE, 
                oneWheelTurn = False,
                then=Stop.BRAKE):
    """
    Turns the robot to the specified absolute angle.
    It calculates if the right or the left turn is the closest
    way to get to the target angle. Can handle negative gyro readings.
    The input should however be in the 0-359 space.
    """
    def _calculateCorrectionForTurn(correction, rotations):
        if rotations * 0.02 < correction:
            return correction - (rotations * 0.02)
        else:
            return 0

    prevStraightSpeed, prevStraightAcceleration, prevTurnSpeed, prevTurnAcceleration = drive_base.settings()
    drive_base.settings(straight_speed=prevStraightSpeed,straight_acceleration=prevStraightAcceleration,
                        turn_rate=speed, turn_acceleration=turn_acceleration)

    currentAngle = hub.imu.heading()
    rotations = currentAngle / 360
    currentAngle = _convertAngleTo360(currentAngle)
    
    if targetAngle >= currentAngle:
        rightTurnDegrees = targetAngle - currentAngle
        leftTurnDegrees = 360 - targetAngle + currentAngle
    else: 
        leftTurnDegrees = currentAngle - targetAngle
        rightTurnDegrees = 360 - currentAngle + targetAngle

    # Figure out the degrees to turn using the correction and the 
    # shortest turning side. Either left or Right.
    degreesToTurn = 0  
    if(forceTurn == FORCETURN_NONE):
        if (rightTurnDegrees < leftTurnDegrees):
            correction = _calculateCorrectionForTurn(right_correction, rotations)
            degreesToTurn = rightTurnDegrees - (correction * rightTurnDegrees)
        else:
            correction = _calculateCorrectionForTurn(left_correction, rotations)
            degreesToTurn = (leftTurnDegrees - (correction * leftTurnDegrees) )* -1

    # If force turn has been specified, then override the calculation for
    # which direction to turn.
    if forceTurn == FORCETURN_LEFT:
        correction = _calculateCorrectionForTurn(left_correction, rotations)
        degreesToTurn = (leftTurnDegrees - (correction * leftTurnDegrees) )* -1
    elif forceTurn == FORCETURN_RIGHT:
        correction = _calculateCorrectionForTurn(right_correction, rotations)
        degreesToTurn = rightTurnDegrees - (correction * rightTurnDegrees)
    
    #print("Before drive_base turn currentAngle = " + str(currentAngle) + " degreesToTurn= " + str(degreesToTurn) + 
    #      "current heading before turn= " + str( hub.imu.heading()) )
    # Use the gyro drive base to turn.
    if oneWheelTurn == False:
        drive_base.turn(degreesToTurn, then=then, wait=True)

    if oneWheelTurn == True:
        if forceTurn == FORCETURN_RIGHT:
            left_motor.run_angle(speed, degreesToTurn * axle_track / wheel_radius)
        elif forceTurn == FORCETURN_LEFT:
            right_motor.run_angle(-1 * speed, degreesToTurn * axle_track / wheel_radius)
        elif degreesToTurn > 0:
            left_motor.run_angle(speed, degreesToTurn * axle_track / wheel_radius)
        elif degreesToTurn < 0:
            right_motor.run_angle(-1 * speed, degreesToTurn * axle_track / wheel_radius)
        else:
            print("ERROR")
        #wait(500)
    
    #print("After turn current heading = " + str( hub.imu.heading()))

    # After the turn we expect that the robot is now correctly pointing in the 
    # direction that we want. Reset the gyro to the angle.
    #hub.imu.reset_heading(targetAngle)

    # Reset the settings to the previous settings.
    drive_base.settings(straight_speed=prevStraightSpeed,straight_acceleration=prevStraightAcceleration,
                        turn_rate=prevTurnSpeed,turn_acceleration=prevTurnAcceleration)

def driveTillDistance(distanceinCM, speed, backward=False, wait=True):
    # Get the drivebase settings
    straight_speed, straight_acceleration, turn_rate, turn_acceleration = getDriveBaseSettings()
    setDriveBaseSettings(speed, straight_acceleration, turn_rate, turn_acceleration)

    distanceToDriveInMM = distanceinCM * 10
    if backward == True:
        distanceToDriveInMM = distanceToDriveInMM * -1

    # Drive forward using the drive base command.
    drive_base.straight(distanceToDriveInMM, wait=wait)

    # restore the default drive base settings
    setDriveBaseSettings(straight_speed, straight_acceleration, turn_rate, turn_acceleration)

def isOnBlackLine():
    light = getReflectedLight()
    if(light<=BLACK_COLOR):
        return True
    return False

def followBlackLinePID(distanceInMM, speed, edge, controlColor = 63, 
                       kp=0.5, ki=0, kd=0, correctionBasedSpeed = False, slowStart = True, 
                       slowDown = True, printDebugMessages=False,
                       additionalFunctionToCall = None,
                       slowSpeedOverride = 100, startcmSLOW=0):

    # Returns: distance to accelerate for, high speed distance, distance to decelerate for.
    def _calculateStartAndEndDistances(distanceInMM, speed):
        accelerationDistanceMM = distanceInMM * 0.2
        midDistanceMM = distanceInMM * 0.6
        decelerationDistanceMM = distanceInMM * 0.2

        if distanceInMM < 20:
            return 0, distanceInMM, 0
        else: 
            return accelerationDistanceMM, midDistanceMM, decelerationDistanceMM

    #prevStraightSpeed, prevStraightAcceleration, prevTurnSpeed, prevTurnAcceleration = drive_base.settings()
    #drive_base.settings(straight_speed=speed,straight_acceleration=100,
    #                    turn_rate=prevTurnSpeed,turn_acceleration=prevTurnAcceleration)

    if (printDebugMessages == True):
        print("Start followBlackLine")
        print("followBlackLine: speed=" + str(speed))
        print("followBlackLine: voltage=" + str(hub.battery.voltage()))
        minSpeed = speed
    
    slowSpeed = slowSpeedOverride
    midSpeed = speed
    accelerationDistanceMM,midDistanceMM,decelerationDistanceMM = _calculateStartAndEndDistances(distanceInMM, speed)

    drive_base.reset(distance = 0, angle=hub.imu.heading())

    # Turn on the drivebase.
    if startcmSLOW != 0:
        speedToUse = 50
    else:
        speedToUse =  slowSpeed
    drive_base.drive(speed = speedToUse, turn_rate = 0)
        
    error_sum = 0
    error = 0

    # Start a stopwatch to measure elapsed time
    watch = StopWatch()
    distanceDriverMM = 0 
    while (distanceDriverMM < distanceInMM):
        distanceDriverMM = drive_base.distance()
        
        # Call the additional passed in function.
        if additionalFunctionToCall != None:
            additionalFunctionToCall(distanceDriverMM)

        value = getValue()
        last_error = error
        error = value - controlColor
        error_sum = error_sum + error
        error_rate = error - last_error

        proportional = kp * error
        integral = ki * error_sum 
        derivative = kd * error_rate

        correction = proportional + integral + derivative

        speedToUse = midSpeed
        if (slowStart == True and drive_base.distance() <= accelerationDistanceMM):
            speedToUse = (((midSpeed - slowSpeed) / accelerationDistanceMM) * drive_base.distance()) + slowSpeed
        if (slowDown == True and drive_base.distance() >= accelerationDistanceMM + midDistanceMM):
            speedToUse = (((slowSpeed - midSpeed) / (distanceInMM - midDistanceMM - accelerationDistanceMM)) * (drive_base.distance() - distanceInMM)) + slowSpeed
        if (startcmSLOW != 0 and distanceDriverMM <= 40):
            speedToUse = 30
        
        turn_rate = correction
        if edge == LINE_FOLLOWER_EDGE_LEFT:
            turn_rate = correction
        else:
            turn_rate = correction * -1

        if (printDebugMessages == True):
            print("followBlackLine," + str(drive_base.distance()) + "," + str(watch.time()) + "," + str(value) + "," +
                str(error) + "," + str(proportional) + "," + str(integral) + "," + str(derivative) + "," + str(turn_rate)+ "," + str(speedToUse))


        drive_base.drive(speed = speedToUse, turn_rate = turn_rate)
        wait(2)

    # This is brake in pybricks. If we want hold then use drive_base.straight(0)
    # drive_base.stop() is coast.
    drive_base.stop()
    drive_base.drive(0, 0)
    drive_base.stop()

errorSum = 0
lastError = 0
def gyroStraightWithDrive(distance, speed, backward = False, targetAngle = 0, 
                          multiplier=4,
                          slowDown=True, printDebugMesssages=False,
                          tillBlackLine = False,
                           tillWhiteLine = False,
                          detectStall = False,
                          stop = Stop.BRAKE,
                          alternativeFunctionToCall = None,
                          slowSpeedOverride = 50,
                          stopWhenLoadAbove = 0,
                          optional_simulator_max_distance_from_wall = None):  # This is the distance of the center of the robot the place on the wall. Used for the bikes.
    # Array of tuples
    #values = []
    global errorSum, lastError
    errorSum = 0
    lastError = 0

    # Drift bias parameters
    DRIFT_DISTANCE_CM = 82
    DRIFT_BIAS_CM = -1
    drift_per_mm = DRIFT_BIAS_CM / (DRIFT_DISTANCE_CM * 10)  # bias per mm

    def _getCorrectionForDrive(targetAngle, distanceDriven, correctionMultiplier, values,
                               printDebugMesssages = False):
        global errorSum, lastError
        currentAngle = hub.imu.heading()
        currentAngle = _convertAngleTo360(currentAngle)

        if targetAngle >= currentAngle:
            rightTurnDegrees = targetAngle - currentAngle
            leftTurnDegrees = 360 - targetAngle + currentAngle
            if (printDebugMesssages == True):
             print("Loop: In targetangle>= cA: " + 
                   " targetAngle = " + str(targetAngle) +
                   " currentAngle = " + str(currentAngle) +
                   " rightTurnDegrees=" + str(rightTurnDegrees) + 
                   " leftTurnDegrees=" + str(leftTurnDegrees) )
         
        else: 
            leftTurnDegrees = currentAngle - targetAngle
            rightTurnDegrees = 360 - currentAngle + targetAngle
            if (printDebugMesssages == True):
             print("Loop: In else targetangle < cA:" 
                   " targetAngle = " + str(targetAngle) +
                   " currentAngle = " + str(currentAngle) +
                   " rightTurnDegrees=" + str(rightTurnDegrees) + 
                   " leftTurnDegrees=" + str(leftTurnDegrees) )
         

        # Figure out the degrees to turn using the correction and the 
        # shortest turning side. Either left or Right.
        degreesToTurn = 0  
        if (rightTurnDegrees < leftTurnDegrees):
            degreesToTurn = rightTurnDegrees 
        else:
            degreesToTurn = -1*leftTurnDegrees

        if (printDebugMesssages == True):
            print("degreesToTurn=" + str(degreesToTurn) + " current_angle=" + str(currentAngle) + 
                " target_angle="+str(targetAngle))
            
        correction = degreesToTurn
        errorSum = errorSum + correction
        averageError = errorSum
        lastError = correction

        kp = correctionMultiplier
        ki = 0
        kd = 0.5
        newCorrection = kp*correction + averageError * ki + (lastError - correction) * kd

        # Drift bias: add a small left turn over 82cm forward
        # Only apply if going forward and distance <= 82cm
        if not backward and distanceDriven <= DRIFT_DISTANCE_CM * 10:
            # Calculate the drift correction in mm, convert to degrees
            # For a left drift, add positive correction (left turn)
            drift_correction_mm = drift_per_mm * distanceDriven  # in cm
            # Convert cm drift to degrees: angle = atan(drift / distance)
            # For small angles, angle (radians) ≈ drift / distance
            # Convert to degrees: degrees = radians * (180/pi)
            if distanceDriven > 0:
                drift_angle_deg = (drift_correction_mm / (distanceDriven / 10)) * (180 / pi)
                newCorrection += drift_angle_deg

        return newCorrection

    def blackStoppingCondition(color_sensor):
        light = _getValueInternal(color_sensor)
        return light <= BLACK_COLOR
    
    def whiteStoppingCondition(color_sensor):
        #light = getReflectedLight()
        light = _getValueInternal(color_sensor)
        return light >= WHITE_COLOR
    
    stopping_condition_function = None
    if (tillBlackLine == True and tillWhiteLine == True):
        raise ValueError("Only tillBlackLine or tillWhiteLine should be true, not both.")
    elif (tillBlackLine == True):
        stopping_condition_function = blackStoppingCondition
    elif (tillWhiteLine == True):
        stopping_condition_function = whiteStoppingCondition

    drive_base.stop()
    drive_base.reset(angle=hub.imu.heading())

    # Calculate all the distances to use for acceleration and deceleration.
    slowSpeed = slowSpeedOverride
    distanceInMM = distance * 10
    accelerationDistanceMM = 0.1 * distanceInMM
    decelerationDistanceMM = 0.2 * distanceInMM

    # If the distance is small just use drive_base.straight.
    if distanceInMM <= 50:
        if backward == True:
            drive_base.straight(distance=distanceInMM * -1)
        else:
            drive_base.straight(distance=distanceInMM)
        return

    # Override the develeration if we dont want to slow down.
    if slowDown == False:
        decelerationDistanceMM = 0

    midDistanceMM = distanceInMM - accelerationDistanceMM - decelerationDistanceMM

    # Calculate the speed reduction per distance travelled. We do
    # this upfront, to enable integer multiplicaation in the loop.
    # deceleration is always a +ve number.
    if (accelerationDistanceMM != 0):
        speedChangePerMM = (speed - slowSpeed) / accelerationDistanceMM

    if (decelerationDistanceMM != 0):
        speedChangePerMMDeceleration = (speed - slowSpeed) / decelerationDistanceMM

    if (printDebugMesssages):
        print(
          "distanceInMM: " + str(distanceInMM) + 
          " accelerationDistanceMM: " + str(accelerationDistanceMM) +
          " midDistanceMM: " + str(midDistanceMM) +
          " decelerationDistanceMM: " + str(decelerationDistanceMM))
    
    distanceDrivenMM = drive_base.distance()
    if backward == True:
        drive_base.drive(speed = -1 * slowSpeed, turn_rate = 0)
    else:
        drive_base.drive(speed = slowSpeed, turn_rate = 0)

    stopCondition = False
    if (printDebugMesssages == True):
        print("distancedrivenMM: " + str(distanceDrivenMM) + " distanceInMM: " +  str(distanceInMM))

    speedToUse = slowSpeed
    timesLoadAboveConsequtively = 0
    while (distanceDrivenMM < distanceInMM):
        #stopCondition = blackStoppingCondition(color_sensor)
        if (tillBlackLine == True or tillWhiteLine == True):
            stopCondition = stopping_condition_function(color_sensor)

        if ((tillBlackLine == True or tillWhiteLine == True) and stopCondition == True):
            break
        elif (detectStall == True and drive_base.stalled() == True):
            break
        elif (stopWhenLoadAbove > 0):
            # When the load is above the threshold 2 times consequtively 
            # stop.
            if (getAverageLoad() > stopWhenLoadAbove):
                print(getAverageLoad())
                timesLoadAboveConsequtively = timesLoadAboveConsequtively + 1
                if timesLoadAboveConsequtively > 2:
                    break
            else:
                timesLoadAboveConsequtively = 0

        distanceDrivenMM = abs(drive_base.distance())
        if (alternativeFunctionToCall != None):
            alternativeFunctionToCall(distanceDrivenMM)

        if (distanceDrivenMM <= accelerationDistanceMM):
            speedToUse = slowSpeed + speedChangePerMM * distanceDrivenMM
        elif (distanceDrivenMM > accelerationDistanceMM and distanceDrivenMM <= accelerationDistanceMM + midDistanceMM):
            speedToUse = speed
        elif (slowDown == True):
            speedToUse = slowSpeed + (speedChangePerMMDeceleration * (distanceInMM - distanceDrivenMM))
        else:
            speedToUse = speed

        correction = _getCorrectionForDrive(targetAngle, distanceDrivenMM, correctionMultiplier = multiplier, values = None,
                                            printDebugMesssages=printDebugMesssages)
        if (backward == True): 
            speedToUse = speedToUse * -1

        if (printDebugMesssages == True):
            print("Loop: distancedrivenMM: " + str(distanceDrivenMM) + " speed: " +  str(speedToUse) + 
                  " correction:" + str(correction) + " speedChangePerMMDeceleration:" + str(speedChangePerMMDeceleration))

        drive_base.drive(speed = speedToUse, turn_rate = correction)

        #if (alternativeFunctionToCall != None):
        #    wait(5)

    drive_base.stop()
    stopDriveBase(stop)
    return stopCondition 


def resetBallPicker():
    ball_picker.reset_angle(0)

PICK_BALLS = 0
DROP_BALLS = 1
SLIDE_BALL = 2
ROVER = 3
def setBallPickerPosition(position = PICK_BALLS,wait = True, speed=1000, stop = Stop.HOLD):
    offset = 5
    if (position == DROP_BALLS):
        ball_picker.run_target(speed = speed, target_angle = 0,wait=wait, then = stop)
    elif(position == PICK_BALLS):
        ball_picker.run_target(speed = speed, target_angle = -180+offset,wait=wait, then = stop)
    elif(position == SLIDE_BALL):
        ball_picker.run_target(speed =  speed, target_angle = -110+offset, wait=wait, then = stop)
    elif(position == ROVER):
        ball_picker.run_target(speed =  speed, target_angle = -165+offset, wait=wait, then = stop)
   

ALL_UP = 0
BELOW_LID = 1
HALF_LIFT_LID = 2
LIFT_LID = 3
DROP_BLOCKS = 4
DROP_BLOCKS_LOWER = 5
PICK_RED = 6
PICK_WHITE = 7
PICK_GREEN = 8
PICK_YELLOW = 9
HOLD_DRONE = 10
GROUND = 11
MIDDLE_DROP_BLOCKS = 12
def setPickersPosition(position = ALL_UP, wait = True, speed=700):
    offset = 40
    # -------- REMEMBER REDUCE TO BRING IT UP AND INCREASE TO BRING IT DOWN ----------

    if (position == ALL_UP):
        pickers.run_target(speed = speed, target_angle = 0,wait=wait)
    elif(position == BELOW_LID):
        pickers.run_target(speed = speed, target_angle = 100+offset, wait=wait)
    elif(position == HALF_LIFT_LID):
        pickers.run_target(speed = speed, target_angle = 70+offset, wait=wait)
    elif(position == LIFT_LID):
        pickers.run_target(speed = speed, target_angle = 35+offset,wait=wait)
    elif(position == MIDDLE_DROP_BLOCKS):
        pickers.run_target(speed = speed, target_angle = 70+offset,wait=wait)
    elif(position == DROP_BLOCKS):
        pickers.run_target(speed = speed, target_angle = 93+offset,wait=wait)
    elif (position == DROP_BLOCKS_LOWER):
        pickers.run_target(speed = speed, target_angle = 98+offset, wait = wait)
    elif (position == HOLD_DRONE):
        pickers.run_target(speed = speed, target_angle = 50+offset, wait = wait)

    elif (position == GROUND):
        pickers.run_target(speed = speed, target_angle = 118+offset, wait = wait)
    
    elif (position == PICK_RED):
        pickers.run_target(speed = speed, target_angle = 86+offset, wait = wait)
    elif (position == PICK_WHITE):
        pickers.run_target(speed = speed, target_angle = 88+offset, wait = wait)
    elif(position == PICK_GREEN):
        pickers.run_target(speed = speed, target_angle = 91+offset,wait=wait)
    elif(position == PICK_YELLOW):
        pickers.run_target(speed = speed, target_angle = 93+offset,wait=wait)
   
    #pickers.hold()

def setPickersPositionPID(position=ALL_UP, waitUntilDone=True, speed=700, kp=1.2, ki=0.01, kd=0.05):
    # Map position to target angle
    offset = 40
    position_angles = {
        ALL_UP: 0+offset,
        BELOW_LID: 100+offset,
        HALF_LIFT_LID: 80+offset,
        LIFT_LID: 20+offset,
        DROP_BLOCKS: 93+offset,
        DROP_BLOCKS_LOWER: 98+offset,
        HOLD_DRONE: 60+offset,
        PICK_RED: 85+offset,
        PICK_WHITE: 86+offset,
        PICK_GREEN: 86+offset,
        PICK_YELLOW: 91+offset,
    }
    target_angle = position_angles.get(position, 0)

    # PID variables
    integral = 2
    last_error = 0
    dt = 10  # ms
    max_output = abs(speed)
    min_output = 100  # minimum speed to overcome stiction

    # Acceleration/deceleration profile
    start_angle = pickers.angle()
    total_move = abs(target_angle - start_angle)
    accel_dist = max(1, min(10, total_move // 3))
    decel_dist = max(1, min(10, total_move // 3))

    def get_profiled_speed(current_angle):
        moved = abs(current_angle - start_angle)
        remaining = abs(target_angle - current_angle)
        # Accelerate
        if moved < accel_dist:
            return min_output + (max_output - min_output) * (moved / accel_dist)
        # Decelerate
        elif remaining < decel_dist:
            return min_output + (max_output - min_output) * (remaining / decel_dist)
        # Cruise
        else:
            return max_output

    # Only call pickers.run() if the speed changes significantly, to avoid unsmooth movement
    last_applied_output = 0
    output_threshold = 5  # Only update if speed changes by more than this

    done = False
    while not done:
        current_angle = pickers.angle()
        error = target_angle - current_angle

        # PID gains adjustment near the end
        if abs(error) < 10:
            kp_local = 0.7
            ki_local = 0.005
            kd_local = 0.12
        else:
            kp_local = kp
            ki_local = ki
            kd_local = kd

        integral += error * (dt / 1000)
        derivative = (error - last_error) / (dt / 1000)
        last_error = error

        # PID output
        output = kp_local * error + ki_local * integral + kd_local * derivative

        # Clamp output to profiled speed (preserve sign)
        profiled_speed = get_profiled_speed(current_angle)
        output = max(-profiled_speed, min(profiled_speed, output))

        # Minimum speed to overcome stiction
        if abs(output) < min_output and abs(error) > 2:
            output = min_output if output > 0 else -min_output

        # If error is large, allow full speed
        if abs(error) > 15:
            output = max(-max_output, min(max_output, output))

        # Only update motor speed if output changes significantly
        if abs(output - last_applied_output) > output_threshold or abs(error) < 1:
            pickers.run(output)
            last_applied_output = output

        # Stop condition
        if abs(error) < 0.5:
            pickers.stop()
            done = True

        wait(dt)

    if waitUntilDone:
        pickers.hold()

def resetPickers():
    pickers_angle = 0
    pickers.reset_angle(pickers_angle)


def followPath():
    global axle_track, wheel_diameter
    wheel_radius_cm = wheel_diameter / 20
    axle_track_cm = axle_track / 10
    # Define path as a list of waypoints
    path = [(0, 0), (25,0), (50, 0), (100, 0)]
    lookahead_distance = 10

    controller = PPPurePursuit(path, lookahead_distance)
    state = PPRobotState(wheel_radius_cm, axle_track_cm)

    while (True):
        #linear_velocity, angular_velocity = controller.control(state, speed=500)
        #print("linear_velocity=" + str(linear_velocity) + " angular_velocity=" + str(angular_velocity))
        #l,r = convertPPLinearAndAngularVelocityToMotorSpeed(linear_velocity, angular_velocity, wheel_radius_cm, axle_track_cm)
        #print("l=" + str(l) + " r=" + str(r))

        # Run the motors at the appropriate speeds
        #left_motor.run(l)
        #right_motor.run(r)    

        # Update the robot's position and heading
        state.updateState()
        wait(1000)
    
# Robot state class
class PPRobotState:
    '''
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

        # reset the imu heading to be zero.
        # reset the motor encoders to zero.
        resetHeading(0)
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
    '''

    def __init__(self, wheel_radius, axle_track):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.left_encoder_reading = 0
        self.right_encoder_reading = 0
        self.wheel_radius = wheel_radius
        self.axle_track = axle_track

        # reset the imu heading to be zero.
        # reset the motor encoders to zero.
        resetHeading(0)
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)

    def updateState(self):
        # read the encoders and update the x and y
        left_current_angle = left_motor.angle() * 2 * 3.14 / 360
        right_current_angle = right_motor.angle() * 2 * 3.14 / 360

        delta_left = left_current_angle - self.left_encoder_reading
        delta_right = right_current_angle - self.right_encoder_reading

        # Update the current encoder readings.
        self.left_encoder_reading = left_current_angle
        self.right_encoder_reading = right_current_angle

        linear_displacement = ((delta_left + delta_right) / 2) * self.wheel_radius
        angular_displacement = ((delta_right - delta_left) * self.wheel_radius ) / self.axle_track
        print("delta_left=" + str(delta_left) + " delta_right=" + str(delta_right) +
            " li=" + str(linear_displacement) + " ad="+ str(angular_displacement))

        if angular_displacement >= 0.01 or angular_displacement <= -0.01:
            R = linear_displacement / angular_displacement
            delta_x = linear_displacement * cos(self.theta + (angular_displacement / 2))
            delta_y = linear_displacement * sin(self.theta + (angular_displacement / 2))

            #print("self.theta=" + str(self.theta) + " ad=" + str(angular_displacement) + " sin(ad)" + str(sin(angular_displacement + self.theta)) + 
            #    " sin(self.theta)" + str(sin(self.theta)))
        else:
            R = 0
            delta_x = linear_displacement * cos(self.theta)
            delta_y = linear_displacement * sin(self.theta)

        # Update the robot's position and heading
        self.x = self.x + delta_x
        self.y = self.y + delta_y
        self.theta = self.theta + angular_displacement

        # Read theta from the IMU
        imu_reading = (hub.imu.heading() * 2 * 3.14) / 360
        print("x=" + str(self.x) + " y=" + str(self.y) + " theta=" + str(self.theta) + " imu_reading=" + str(imu_reading))

class stall_detect:
    def getLoad():
        left_load = abs(left_motor.load())
        right_load = abs(right_motor.load())
        currLoad = int((left_load + right_load) / 2)
        return currLoad

    # stalls based on load
    def load_three_readings(max_load = 100, min_stopping_condition = 40, stop_motors=True, numObs = 10, debug = False):
        # Initialize the load array with 40s
        loads = []
        for i in range(numObs):
            loads.append(40)

        cur_index = 0
        while drive_base.done() == False:
            currLoad = abs((abs(left_motor.load()) + abs(right_motor.load()) )/ 2)
            if (currLoad > min_stopping_condition):
                loads[cur_index] = currLoad
                cur_index = cur_index + 1
                if (cur_index == numObs):
                    cur_index = 0
            avg_load = sum(loads)/len(loads)
            load = avg_load

            if load  > max_load: # if there is extra load
                if debug == True:
                    print("Stopping stall detection with " + str(load) + " load.") # print debug messages

                # stop the motors
                if (stop_motors == True):
                    left_motor.stop()
                    right_motor.stop()
                    drive_base.stop()

                # exit the loop
                return True

            else: # if there is no extra load
                if debug == True:
                    print("Continuing stall detection with " + str(load) + " load.") # print debug messages

                continue # keep running the loop

        return False

    def bucket_load(bucketmotor,backwardcorrection,max_load = 100, debug=False):
        
        while bucketmotor.done() == False:
            load=bucketmotor.load()
            if(load>max_load):
                if(backwardcorrection==True):
                      drive_base.straight(-5)
                else:
                       drive_base.straight(5)
                   
                if debug == True:
                        print("Moving backwards as Load detected at: " + str(load))
               
            else:
                    if debug == True:
                        print("No load detected: " + str(load))
    
    # stalls based on load
    def load(max_load = 100, min_stopping_condition = 40, stop_motors=True, debug = False):

        while drive_base.done() == False: # while either motor is still moving
            #load = stall_detect.getCurrentLoad(3, minAllowedLoad = min_stopping_condition)
            load = stall_detect.getLoad()
            if load  > max_load: # if there is extra load
                if debug == True:
                    print("Stopping stall detection with " + str(load) + " load.") # print debug messages

                # stop the motors
                if (stop_motors == True):
                    left_motor.stop()
                    right_motor.stop()

                # exit the loop
                break

            else: # if there is no extra load
                if debug == True:
                    print("Continuing stall detection with " + str(load) + " load.") # print debug messages

                continue # keep running the loop

    # stalls based on running average of load
    def avg_load(max_load_change, minValidLoad = 30, minObservationsRequired = 5, min_dist = 0, debug = False):
        loadArr = []

        main_counter = 0
        if min_dist > 0:
            drive_base.reset()

        while drive_base.done() == False: # while the code is still running
            # calculate load
            currLoad = stall_detect.getCurrentLoad(3, minValidLoad)

            if(currLoad < minValidLoad):
                continue
            
            if(len(loadArr)>=minObservationsRequired):
                # calculate average of running average
                load_sum = 0
                for i in range(len(loadArr)):
                    load_sum = load_sum + loadArr[i]

                avgLoad = int(load_sum / (1+len(loadArr)))

                # if the load is greater than 50% more than the running average
                if len(loadArr) >= minObservationsRequired and currLoad >= int(avgLoad * (max_load_change + 1)) and currLoad > minValidLoad and drive_base.distance() > min_dist:

                    # stop the motors
                    left_motor.stop()
                    right_motor.stop()

                    if debug == True:
                        print("Stopping stall detection with " + str(currLoad) + " load and " + str(int(avgLoad * (max_load_change + 1))) + " stopping condition and " + str(main_counter) + " readings") # print debug messages

                    # exit the function
                    break

                else:
                    if debug == True:
                        print("Continuing stall detection with " + str(currLoad) + " load and " + str(int(avgLoad * (max_load_change + 1))) + " stopping condition." + str(main_counter) + " readings") # print debug messages
                        print(loadArr)

                loadArr[main_counter % minObservationsRequired] = currLoad
            else:
                loadArr.append(currLoad)
                if debug == True:
                    print("not enough observations. Adding {} to the array.".format(currLoad))

            main_counter += 1
            wait(5)

    def getCurrentLoad(numObs, minAllowedLoad, delayBetweenReadingsMs=1):
        totalLoad = 0 #[0, 0, 0]
        for i in range(0, numObs):
            currLoad = 0
            while(currLoad < minAllowedLoad):
                left_load = abs(left_motor.load())
                right_load = abs(right_motor.load())
                currLoad = int((left_load + right_load) / 2)
                #print("Current load {} is less than min {}".format(currLoad, minAllowedLoad))
            # totalLoad[i] = currLoad
            totalLoad = totalLoad + currLoad
            #wait(delayBetweenReadingsMs)
        return int(totalLoad/numObs)

    # stalls based on angle change
    def angle_change(max_iterations = 5, deadzone = 10, debug = False):
        # define reading arrays
        right_readings = []
        left_readings = []

        while drive_base.done() == False: # while either motor is still moving
            if int((len(right_readings) + len(left_readings)) / 2) > 5: # if there are more than 5 readings
                # append new readings
                right_readings.append(right_motor.angle())
                left_readings.append(left_motor.angle())

                # calculate the motor speed for proportional calculations
                right_speed = right_motor.speed()
                left_speed = left_motor.speed()
                speed = int((right_speed + left_speed) / 2)

                # calculate the reading diff
                right_reading_diff = right_readings[-1] - right_readings[-1 * max_iterations]
                left_reading_diff = left_readings[-1] - left_readings[-1 * max_iterations]
                reading_diff = int((right_reading_diff + left_reading_diff) / 2)

                if reading_diff < int((deadzone * speed) / 100): # if the readings haven't changed enough
                    # stop the motors
                    right_motor.stop()
                    left_motor.stop()
                    
                    if debug == True:
                        print("Stopping stall detection with angle change of " + str(reading_diff) + " degrees.") # print debug messages

                    # exit the loop
                    break

                else:
                    if debug == True:
                        print("Continuing stall detection with angle change of " + str(reading_diff) + " degrees.")

            else: # if there are less than 5 readings
                # append new readings
                right_readings.append(right_motor.angle())
                left_readings.append(left_motor.angle())

                
def flushUsingStallDetect(distance, max_load=100, speed=200, backward=False, acceleration=500, min_dist=0):
    prev_straight_speed, prev_straight_acceleration, prev_turn_rate, prev_turn_acceleration = drive_base.settings()
    drive_base.settings(straight_speed=speed, straight_acceleration=acceleration,
                        turn_rate=speed, turn_acceleration=acceleration)
    if backward:
        distance = distance * -1
    drive_base.straight(distance=distance * 10, wait=False)
    stall_detect.load(max_load=max_load, debug=False, min_dist=min_dist)

    # Reset the value to what they were.
    drive_base.settings(prev_straight_speed, prev_straight_acceleration, prev_turn_rate, prev_turn_acceleration)
    drive_base.stop()

# Modified stall_detect.load to havew min_dist
# Added min_dist parameter with default 0
def load(max_load=100, min_stopping_condition=40, stop_motors=True, debug=False, min_dist=0):
    while drive_base.done() == False:
        load = stall_detect.getLoad()
        # Only trigger if min_dist is met

        
        if abs(drive_base.distance()) / 10 >= abs(min_dist) and load > max_load:
            if debug:
                print("Stopping stall detection with " + str(load) + " load.")
            if stop_motors:
                left_motor.stop()
                right_motor.stop()
            break
        else:
            if debug:
                print("Continuing stall detection with " + str(load) + " load.")
            continue
    #print("Final dist = " + str(drive_base.distance()) + " load= " + str(load))
stall_detect.load = load

# Pure Pursuit Controller
class PPPurePursuit:
    def __init__(self, path, lookahead_distance):
        self.path = path
        self.lookahead_distance = lookahead_distance

    def find_lookahead_point(self, state):
        closest_dist = float('inf')
        closest_point = None
        for point in self.path:
            dist = sqrt((point[0] - state.x)**2 + (point[1] - state.y)**2)
            if dist >= self.lookahead_distance and dist < closest_dist:
                closest_dist = dist
                closest_point = point
        return closest_point

    def control(self, state, speed):
        lookahead_point = self.find_lookahead_point(state)
        if lookahead_point is None:
            print("lost point")
            return 0, 0  # Goal reached or path issue

        # Transform lookahead point to robot coordinate frame
        dx = lookahead_point[0] - state.x
        dy = lookahead_point[1] - state.y

        # Coordinate transformation
        x_local = cos(-state.theta) * dx - sin(-state.theta) * dy
        y_local = sin(-state.theta) * dx + cos(-state.theta) * dy

        # Compute curvature
        print("y_local=" + str(y_local) + " lookahead_distance=" + str(self.lookahead_distance))
        curvature = (2 * y_local) / (self.lookahead_distance ** 2)

        # Velocity commands
        linear_velocity = speed
        angular_velocity = linear_velocity * curvature

        return linear_velocity, angular_velocity


def convertPPLinearAndAngularVelocityToMotorSpeed(linear_velocity, angular_velocity, wheel_base, wheel_radius):
    w_left = (linear_velocity + (angular_velocity * wheel_base / 2)) / wheel_radius
    w_right = (linear_velocity - (angular_velocity * wheel_base / 2)) / wheel_radius

    return w_left, w_right

def medianOfAList(listOfNumbers):
    sorted_data = sorted(listOfNumbers)
    n = len(sorted_data)
    if n % 2 == 1:  # Odd number of elements
        middle_index = n // 2
        median = sorted_data[middle_index]
    else:  # Even number of elements
        middle_index_1 = n // 2 - 1
        middle_index_2 = n // 2
        median = (sorted_data[middle_index_1] + sorted_data[middle_index_2]) / 2
    return median

def removeAValueFromList(list, value1, value2=None):
    newList = [char for char in list if char != value1]
    if value2 is None:
        return newList
    else:
        return [char for char in newList if char != value2]

def curve_with_accel_options(radius, angle, speed=300, acceleration=450, deceleration=0,
                             dont_accelerate=False, dont_decelerate=False,
                             optional_simulator_end_point=None, optional_simulator_end_angle=None):
    if deceleration == 0:
        deceleration = acceleration

    prev_straight_speed, prev_straight_acceleration, prev_turn_rate, prev_turn_acceleration = drive_base.settings()

    # If dont_accelerate or dont_decelerate, just set the speed and accel, but do NOT stop at the end
    if dont_accelerate or dont_decelerate:
        drive_base.settings(straight_speed=speed, straight_acceleration=(acceleration, deceleration),
                            turn_rate=speed, turn_acceleration=(acceleration, deceleration))
        drive_base.curve(radius=radius, angle=angle)
        # Restore settings, but do not stop the drivebase
        drive_base.settings(prev_straight_speed, prev_straight_acceleration, prev_turn_rate, prev_turn_acceleration)
    else:
        drive_base.settings(straight_speed=speed, straight_acceleration=(acceleration, deceleration),
                            turn_rate=speed, turn_acceleration=(acceleration, deceleration))
        drive_base.curve(radius=radius, angle=angle)
        drive_base.settings(prev_straight_speed, prev_straight_acceleration, prev_turn_rate, prev_turn_acceleration)
        drive_base.stop()

def gyroStraightWithDrive_accel_options(distance, speed, backward=False, targetAngle=0,
                                        multiplier=4, slowDown=True, printDebugMesssages=False,
                                        tillBlackLine=False, detectStall=False, stop=Stop.BRAKE,
                                        alternativeFunctionToCall=None, slowSpeedOverride=50,
                                        stopWhenLoadAbove=0, optional_simulator_max_distance_from_wall=None,
                                        dont_accelerate=False, dont_decelerate=False):
    global errorSum, lastError
    errorSum = 0
    lastError = 0

    DRIFT_DISTANCE_CM = 82
    DRIFT_BIAS_CM = -1
    drift_per_mm = DRIFT_BIAS_CM / (DRIFT_DISTANCE_CM * 10)

    def _getCorrectionForDrive(targetAngle, distanceDriven, correctionMultiplier, values,
                                printDebugMesssages=False):
        global errorSum, lastError
        currentAngle = hub.imu.heading()
        currentAngle = _convertAngleTo360(currentAngle)

        if targetAngle >= currentAngle:
            rightTurnDegrees = targetAngle - currentAngle
            leftTurnDegrees = 360 - targetAngle + currentAngle
        else:
            leftTurnDegrees = currentAngle - targetAngle
            rightTurnDegrees = 360 - currentAngle + targetAngle

        degreesToTurn = rightTurnDegrees if (rightTurnDegrees < leftTurnDegrees) else -1 * leftTurnDegrees
        correction = degreesToTurn
        errorSum = errorSum + correction
        averageError = errorSum
        lastError = correction

        kp = correctionMultiplier
        ki = 0
        kd = 0.5
        newCorrection = kp * correction + averageError * ki + (lastError - correction) * kd

        if not backward and distanceDriven <= DRIFT_DISTANCE_CM * 10:
            if distanceDriven > 0:
                drift_angle_deg = (drift_per_mm * distanceDriven / (distanceDriven / 10)) * (180 / pi)
                newCorrection += drift_angle_deg

        return newCorrection

    def blackStoppingCondition():
        light = getValue()
        return light <= BLACK_COLOR

    drive_base.stop()
    drive_base.reset(angle=hub.imu.heading())

    slowSpeed = slowSpeedOverride
    distanceInMM = distance * 10
    accelerationDistanceMM = 0.1 * distanceInMM
    decelerationDistanceMM = 0.2 * distanceInMM

    if distanceInMM <= 50:
        if backward:
            drive_base.straight(distance=distanceInMM * -1)
        else:
            drive_base.straight(distance=distanceInMM)
        return

    if slowDown == False:
        decelerationDistanceMM = 0

    midDistanceMM = distanceInMM - accelerationDistanceMM - decelerationDistanceMM

    if accelerationDistanceMM != 0:
        speedChangePerMM = (speed - slowSpeed) / accelerationDistanceMM
    if decelerationDistanceMM != 0:
        speedChangePerMMDeceleration = (speed - slowSpeed) / decelerationDistanceMM

    distanceDrivenMM = drive_base.distance()
    if backward:
        drive_base.drive(speed=-1 * slowSpeed, turn_rate=0)
    else:
        drive_base.drive(speed=slowSpeed, turn_rate=0)

    stopCondition = False
    speedToUse = slowSpeed
    timesLoadAboveConsequtively = 0

    while distanceDrivenMM < distanceInMM:
        stopCondition = blackStoppingCondition()
        if tillBlackLine and stopCondition:
            break
        elif detectStall and drive_base.stalled():
            break
        elif stopWhenLoadAbove > 0:
            if getAverageLoad() > stopWhenLoadAbove:
                timesLoadAboveConsequtively += 1
                if timesLoadAboveConsequtively > 2:
                    break
            else:
                timesLoadAboveConsequtively = 0

        distanceDrivenMM = abs(drive_base.distance())
        if alternativeFunctionToCall is not None:
            alternativeFunctionToCall(distanceDrivenMM)

        if dont_accelerate and dont_decelerate:
            speedToUse = speed
        elif dont_accelerate:
            if distanceDrivenMM > accelerationDistanceMM and distanceDrivenMM <= accelerationDistanceMM + midDistanceMM:
                speedToUse = speed
            elif slowDown and distanceDrivenMM > accelerationDistanceMM + midDistanceMM:
                speedToUse = slowSpeed + (speedChangePerMMDeceleration * (distanceInMM - distanceDrivenMM))
            else:
                speedToUse = speed
        elif dont_decelerate:
            if distanceDrivenMM <= accelerationDistanceMM:
                speedToUse = slowSpeed + speedChangePerMM * distanceDrivenMM
            else:
                speedToUse = speed
        else:
            if distanceDrivenMM <= accelerationDistanceMM:
                speedToUse = slowSpeed + speedChangePerMM * distanceDrivenMM
            elif distanceDrivenMM > accelerationDistanceMM and distanceDrivenMM <= accelerationDistanceMM + midDistanceMM:
                speedToUse = speed
            elif slowDown:
                speedToUse = slowSpeed + (speedChangePerMMDeceleration * (distanceInMM - distanceDrivenMM))
            else:
                speedToUse = speed

        correction = _getCorrectionForDrive(targetAngle, distanceDrivenMM, correctionMultiplier=multiplier, values=None,
                                            printDebugMesssages=printDebugMesssages)
        if backward:
            speedToUse = speedToUse * -1

        if printDebugMesssages:
            print("Loop: distancedrivenMM: " + str(distanceDrivenMM) + " speed: " + str(speedToUse) +
                    " correction:" + str(correction))

        drive_base.drive(speed=speedToUse, turn_rate=correction)

    drive_base.stop()
    stopDriveBase(stop)
    return stopCondition