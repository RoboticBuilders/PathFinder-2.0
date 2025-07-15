# To Run a simulator
# 1. open helpers.py and uncomment simulation_utils import. Comment the utils import.
# 2. Go the bottom of this file and uncomment the runSimulator()

from helpers import *
from path_planner import PathPlanner

def goFromHomeToBallPit():
    angle = 180
    # get out of the home area
    gyroStraightWithDrive(distance=13,speed=900,targetAngle=angle, backward=True, slowDown=False)
    drive_base.settings(800, 800, 800, 800)
    curve(radius=-165,angle = 180, speed=400)
    flushUsingStallDetect(distance = 3, backward = True)
    hub.imu.reset_heading(0)
    #angle = -90
    #gyroStraightWithDrive(distance=17,speed=500,targetAngle=angle,tillBlackLine=True)


def getBallsFromBallPit():
    angle = 0
    speed = 200
    acceleration = 200
    drive_base.settings(speed, acceleration, speed, acceleration)
    setPickersPosition(position=PICK_GREEN, speed=300, wait=False)
    gyroStraightWithDrive(distance=8,speed=200,targetAngle=angle)
    setBallPickerPosition(position=PICK_BALLS, speed=300, wait=True)

    # Now drive into the pit to pick the first ball
    wait(50)
    drive_base.straight(-130)
    drive_base.straight(50)
    setBallPickerPosition(position=SLIDE_BALL, speed=100, wait = False)
    drive_base.straight(50)
    #ball_picker.hold()
    #wait(100)
    setBallPickerPosition(PICK_BALLS,speed=150, wait = False)
 
    angle = 0
    turnToAngle(targetAngle=angle, speed=100)
    gyroStraightWithDrive(distance=12, speed=100, backward=True,targetAngle=angle)
    wait(100)
    #ball_picker.run(speed=100)
    
    setPickersPosition(ALL_UP, wait = False)
    gyroStraightWithDrive(distance=5, speed=250, targetAngle=angle)
    #ball_picker.hold()
    #setBallPickerPosition(position = SLIDE_BALL,speed=300, wait = True)
  
    ball_picker.run_target(speed =  500, target_angle = -140, wait=False)
    wait(100)


def dropBalls():
    # angle = -90
    # turnToAngle(targetAngle=angle, speed=600)
    # gyroStraightWithDrive(distance=44, speed=350, targetAngle=angle)

    # angle=180
    # turnToAngle(targetAngle=angle, speed=500)
    # setPickersPosition(BELOW_LID)
    # flushUsingStallDetect(distance=26, max_load=110)

    curve_with_accel_options(radius = 110, angle = -90, speed = 400, dont_decelerate = True)
    ball_picker.run_target(speed = 100, target_angle = -110, wait=False)
    # gyroStraightWithDrive_accel_options(distance = 22, speed = 800, targetAngle = -90, slowDown = False, slowSpeedOverride = 250, dont_accelerate = True, dont_decelerate = True)
    drive_base.settings(800, 800, 800, 800)
    drive_base.straight(205)
    curve_with_accel_options(radius = 110, angle = -90, speed = 800, dont_accelerate = True, dont_decelerate = True)
    pickers.run_target(speed = 500, target_angle = 120, wait=True)
    '''
    flushUsingStallDetect(distance=14, max_load=110)
    # resetHeading(180)
    wait(100)

    drive_base.straight(-20)
    '''

    # gyroStraightWithDrive(distance = 6, speed = 300, targetAngle = 180)
    drive_base.settings(200, 200, 200, 200)
    drive_base.straight(52)

    setPickersPosition(HALF_LIFT_LID, speed=500, wait=True)
    gyroStraightWithDrive(distance=4, speed=600, targetAngle=180, slowDown=False)
    setPickersPosition(LIFT_LID, speed=1000, wait = False)
    gyroStraightWithDrive(distance=6, speed=600, targetAngle=180, slowDown=False)
    setBallPickerPosition(DROP_BALLS, speed= 200, wait = True, stop = Stop.COAST_SMART)


def dropBallsAxleBucket():
    # angle = -90
    # turnToAngle(targetAngle=angle, speed=600)
    # gyroStraightWithDrive(distance=44, speed=350, targetAngle=angle)

    # angle=180
    # turnToAngle(targetAngle=angle, speed=500)
    # setPickersPosition(BELOW_LID)
    # flushUsingStallDetect(distance=26, max_load=110)

    curve_with_accel_options(radius = 110, angle = -90, speed = 400, dont_decelerate = True)
    ball_picker.run_target(speed = 100, target_angle = -110, wait=False)
    # gyroStraightWithDrive_accel_options(distance = 22, speed = 800, targetAngle = -90, slowDown = False, slowSpeedOverride = 250, dont_accelerate = True, dont_decelerate = True)
    drive_base.settings(800, 800, 800, 800)
    gyroStraightWithDrive(distance=20, speed=800, targetAngle=-90)
    curve_with_accel_options(radius = 110, angle = -90, speed = 800, dont_accelerate = True, dont_decelerate = True)
    turnToAngle(targetAngle= 180)
    setPickersPosition(BELOW_LID)
    gyroStraightWithDrive(distance=20, speed=200, targetAngle=180)
    # resetHeading(180) 
    wait(100)

    drive_base.straight(-15)
    setPickersPosition(HALF_LIFT_LID)
    wait(50)
    drive_base.straight(80)
    setPickersPosition(LIFT_LID, speed = 200)
    drive_base.straight(10)
    setBallPickerPosition(DROP_BALLS, speed=200)


def doRoverAfterBalls():
    #angle =180
    #gyroStraightWithDrive(distance=49, speed=850, backward=True, targetAngle=angle)

    #angle=-90
    #turnToAngle(targetAngle=angle, speed=400)
    drive_base.settings(800, 800, 800, 800)
    # gyroStraightWithDrive_accel_options(distance=22, speed=650, backward=True, targetAngle=180, dont_decelerate = True)
    drive_base.straight(-240)
    setBallPickerPosition(ROVER, speed=600, wait = False)
    curve_with_accel_options(radius=-205, angle=-90, speed=850, dont_accelerate = True, dont_decelerate = True)
    #gyroStraightWithDrive(distance=5, speed=650, backward=True, targetAngle=-90)
    # turnToAngle(targetAngle=-70, speed=800)


def goToBlocks():
    # angle = -70
    # setBallPickerPosition(DROP_BALLS, wait = False)
    # gyroStraightWithDrive(distance=27, speed=800, targetAngle=angle)
    # setPickersPosition(ALL_UP, wait = False)

    # angle = 180
    # turnToAngle(targetAngle=angle, speed=800)
    # gyroStraightWithDrive(distance=35, speed=500, backward=True, targetAngle=angle, tillBlackLine= True)
    # gyroStraightWithDrive(distance=12, speed=700, backward=True, targetAngle=angle)

    # angle = 90
    # turnToAngle(targetAngle=angle, speed=750)
    # drive_base.straight(-100)
    # drive_base.stop()
    # hub.imu.reset_heading(90)
    # setPickersPosition(ALL_UP)

    drive_base.settings(800, 800, 800, 800)
    drive_base.straight(150)
    setBallPickerPosition(DROP_BALLS, wait = False)
    curve_with_accel_options(radius = 120, angle = 90, speed = 800, dont_accelerate = True, dont_decelerate = True)
    setPickersPosition(ALL_UP, wait = False)
    turnToAngle(0)
    #waitForRightButtonPress()
    if gyroStraightWithDrive(distance=35, speed=500, targetAngle=0, tillBlackLine= True,slowDown=False)  == False:
        print("Missed line catch before blocks scanning")
    drive_base.straight(130)
    drive_base.turn(90)
    #flushUsingStallDetect(distance = 10, backward = True)
    gyroStraightWithDrive(distance=10,targetAngle=90,backward=True,speed=600)
    wait(100)
    resetHeading(90)


def detectBlocksAndDecideWhichOneToStartPickupFrom(printDebugMessages=False):
    # 1. Starts from home and detects the color of the first 4 blocks
    # 2. Conclues the color of the last two blocks
    # 3. Determines the starting block for the three blocks to pick up first.
    # 4. Returns the starting block.
    global blockDict
    global lastBlockNumber
    global numberOfBluesFound
    global detectBlocksPrintMessageGlobal
    global hueValuesList
    global saturationValuesList
    global cleanSaturationValuesList
    global valueValuesList


    def _resetGlobals():
        global blockDict
        global lastBlockNumber
        global numberOfBluesFound
        global detectBlocksPrintMessageGlobal
        global hueValuesList
        global saturationValuesList
        global valueValuesList
        global cleanSaturationValuesList
        global distanceBetweenBlocksMM
        global blockSizeMM


        blockDict = {}
        lastBlockNumber = 1
        numberOfBluesFound = 0
        detectBlocksPrintMessageGlobal = False
        hueValuesList = []
        saturationValuesList = []
        valueValuesList = []
        cleanSaturationValuesList = []
        distanceBetweenBlocksMM = 63
        blockSizeMM = 33


    def _determineBlockColor(currentBlockNumber):
        global blockDict
        global numberOfBluesFound
        global detectBlocksPrintMessageGlobal
        global hueValuesList
        global saturationValuesList
        global valueValuesList
        global cleanSaturationValuesList


        # Check for zero in division
       
        if len(hueValuesList) != 0:
            medianHue = medianOfAList(hueValuesList)
            medianSaturation = medianOfAList(saturationValuesList)
            avgHue = sum(hueValuesList)/len(hueValuesList)
            avgSaturation = sum(saturationValuesList)/len(saturationValuesList)
            avgValue =  sum(valueValuesList)/len(valueValuesList)
            medianValue = medianOfAList(valueValuesList)

            if len(cleanSaturationValuesList) != 0:
                cleanAvgSaturation = sum(cleanSaturationValuesList)/len(cleanSaturationValuesList)
                #print(cleanSaturationValuesList)
                 
            else:
                cleanAvgSaturation = 0
        else:
            return
        if avgValue == 0:
            pass
        if(medianHue > 20 and medianHue <61):
            blockDict["yellow"] = currentBlockNumber - 1
        elif (medianHue > 135 and medianHue < 175):
            blockDict["green"] = currentBlockNumber - 1
        elif (medianValue > 0 and medianHue > 175 and medianHue < 260) or ( (avgSaturation <= 20 and medianValue > 10)):
            blockDict["white"] = currentBlockNumber - 1
        elif (medianHue > 300):
            blockDict["red"] = currentBlockNumber - 1
           
        '''
        if avgValue == 0:
            pass
        elif cleanAvgSaturation < 40:
            blockDict["white"] = currentBlockNumber - 1
        elif (medianHue > 240 or medianHue < 30) and medianSaturation > 50:
            blockDict["red"] = currentBlockNumber - 1
        elif 130 <= medianHue <= 190 and medianSaturation > 50:
            blockDict["green"] = currentBlockNumber - 1
        elif 30 <= medianHue <= 90 and medianSaturation > 50:
            blockDict["yellow"] = currentBlockNumber - 1


        '''
       
        if (detectBlocksPrintMessageGlobal == True):
            print("currentBlock = " + str(currentBlockNumber) +
                    ",MedianHue = " + str(medianHue) +
                    ",AvgHue = " + str(avgHue) +
                    ",MedianSaturation = " + str(medianSaturation) +
                    ",AvgSaturation = " + str(avgSaturation) +
                    ",CleanAvgSaturation = " + str(cleanAvgSaturation) +
                    ",AvgValue = " + str(avgValue))


    def _gatherDataFunction(distanceInMM):
        global lastBlockNumber
        global numberOfBluesFound
        global blockDict
        global detectBlocksPrintMessageGlobal
        global hueValuesList
        global saturationValuesList
        global valueValuesList
        global cleanSaturationValuesList




        distanceBetweenBlocksMM = 63
        blockSizeMM = 32
        oneBlockWithGapMM = distanceBetweenBlocksMM + blockSizeMM


        # We gather values when we are 20mm into the block and for 20mm
        # This distances are dependent upon where the block starts
        # and the speed of the robot. This is at 150 speed.
        currentBlockNumber = int(distanceInMM / oneBlockWithGapMM) + 1


        # This means that the block changed.
        if currentBlockNumber > lastBlockNumber:    
            # This method uses globals to determine the current block
            # color and then stores in the blockDict.  
            _determineBlockColor(currentBlockNumber)  
            # Reset all the per-block variables.
            hueValuesList = []
            saturationValuesList = []
            valueValuesList = []


            cleanSaturationValuesList = []


        if ((distanceInMM > ((currentBlockNumber - 1 ) * oneBlockWithGapMM) + 10) and
            (distanceInMM < ((currentBlockNumber - 1 ) * oneBlockWithGapMM + 32))):
            hsv = block_color.hsv()


            if (hsv.h) != 0:
                hueValuesList.append(hsv.h)
                saturationValuesList.append(hsv.s)
                valueValuesList.append(hsv.v)


            #if hsv.h != 240 and hsv.h != 180:
            #    cleanSaturationValuesList.append(hsv.s)


            if (detectBlocksPrintMessageGlobal == True):
                print("d=" + str(distanceInMM) + ",blocknumber=" + str(currentBlockNumber) +
                    ",h=" + str(hsv.h) +
                    ",s=" + str(hsv.s) +
                    ",v=" + str(hsv.v))

        lastBlockNumber = currentBlockNumber

    _resetGlobals()
    detectBlocksPrintMessageGlobal = printDebugMessages
    #waitForLeftButtonPress()  


    speed = 150
    drive_base.settings(800, 800, 800, 800)
    drive_base.straight(270)
   
    totalDistanceInMM = 5 * distanceBetweenBlocksMM + 6 * blockSizeMM + 70
    gyroStraightWithDrive(distance=(totalDistanceInMM/10), speed=speed, targetAngle=90, alternativeFunctionToCall= _gatherDataFunction, backward=False)
    print(blockDict)
    return blockDict


'''def OLD_CODE_pushDronetoHome():
    angle = 45
    turnToAngle(targetAngle=angle,speed=400)
    gyroStraightWithDrive(distance=10, speed=800, targetAngle=angle,backward=False)
    angle = 180
    turnToAngle(targetAngle=angle,speed=400)
    gyroStraightWithDrive(distance=55, speed=1000, targetAngle=angle,backward=True)
    gyroStraightWithDrive(distance=55, speed=1000, targetAngle=angle,backward=False)'''


def pickupRedYellowOLD(block_dict):
    def _pickupYellowFirst():
        dist = 35 + 9.5 * (5 - yellowIndex)
        setPickersPosition(position=PICK_YELLOW, speed=1000)
        drive_base.straight(dist * 10)
        drive_base.turn(-65)
        drive_base.straight(80)  # Reduced from 200 to 80
        setPickersPosition(position=ALL_UP, speed=1000)

    def _pickupYellowSecond():
        if redIndex <= 3:
            drive_base.straight(-80)
            drive_base.turn(120)
            gyroStraightWithDrive(distance=50, speed=500, backward=True, targetAngle=-90, tillBlackLine=True)
        else:
            drive_base.straight(-80)
            drive_base.turn(120)
            gyroStraightWithDrive(distance=50, speed=500, targetAngle=-90, tillBlackLine=True)

        if yellowIndex <= 3:
            dist = 9.5 * (3 - yellowIndex)
            setPickersPosition(position=PICK_YELLOW, speed=1000)
            drive_base.straight(dist * 10)
            drive_base.turn(-65)
            drive_base.straight(80)  # Reduced from 200 to 80
            setPickersPosition(position=ALL_UP, speed=1000)
        else:
            dist = 70 + 9.5 * (yellowIndex - 3)
            setPickersPosition(position=PICK_YELLOW, speed=1000)
            drive_base.straight(-10 * dist)
            drive_base.turn(-65)
            drive_base.straight(80)  # Reduced from 200 to 80
            setPickersPosition(position=ALL_UP, speed=1000)

    def _pickupRedFirst():
        dist = 35 + 9.5 * (7 - redIndex)
        setPickersPosition(position=PICK_RED, speed=1000)
        drive_base.straight(10 * dist)
        drive_base.turn(-120)
        drive_base.straight(80)  # Reduced from 200 to 80
        setPickersPosition(position=ALL_UP, speed=1000)

    def _pickupRedSecond():
        if yellowIndex <= 3:
            drive_base.straight(-100)
            drive_base.turn(65)
            gyroStraightWithDrive(distance=50, speed=500, backward=True, targetAngle=-90, tillBlackLine=True)
        else:
            drive_base.straight(-100)
            drive_base.turn(65)
            gyroStraightWithDrive(distance=50, speed=500, targetAngle=-90, tillBlackLine=True)

        if redIndex <= 3:
            dist = 9.5 * (4 - redIndex)
            drive_base.straight(dist * 10)
            drive_base.turn(-120)
            setPickersPosition(position=PICK_RED, speed=1000)
            drive_base.straight(80)  # Reduced from 200 to 80
            setPickersPosition(position=ALL_UP, speed=1000)
        else:
            dist = 70 + 9.5 * (redIndex - 2)
            drive_base.straight(-10 * dist)
            drive_base.turn(-120)
            setPickersPosition(position=PICK_RED, speed=1000)
            drive_base.straight(80)  # Reduced from 200 to 80
            setPickersPosition(position=ALL_UP, speed=1000)

    yellowIndex = block_dict["yellow"]
    redIndex = block_dict["red"]

    if yellowIndex > redIndex:
        _pickupYellowFirst()
        _pickupRedSecond()
    else:
        _pickupRedFirst()
        _pickupYellowSecond()


def pickupRedYellow(block_dict):
    def pickFirstBlock(block, direction):
        OffsetDistanceMM = 250
        distanceBetweenBlocksMM = 63
        blockSizeMM = 32
        dirOffset = 0

        if direction == "left":
            # FOR YELLOW
            distMM = (OffsetDistanceMM + ((6 - block) * (distanceBetweenBlocksMM + blockSizeMM))) - dirOffset
            angle = -155
            pos = PICK_YELLOW
            dist = 80
        
        if direction == "right":
            # ---- RIGHT ARM IS LOWER -----
            # FOR RED
            distMM = (OffsetDistanceMM + ((6 - block) * (distanceBetweenBlocksMM + blockSizeMM))) + dirOffset
            angle = 163
            pos = PICK_RED
            dist = 100

        
        drive_base.straight(distMM)
        setPickersPosition(pos)
        turnToAngle(targetAngle=angle, speed=300)
        drive_base.straight(dist)  # Reduced from 200 to 80
        # waitForRightButtonPress()
        setPickersPosition(position=ALL_UP, speed=300)
        drive_base.straight(-90)
        turnToAngle(targetAngle=-90, speed=300)

    def pickSecondBlock(currentBlock, BlockToGoTo, direction):
        distanceBetweenBlocksMM = 63
        blockSizeMM = 32
        dirOffset = 0

        if direction == "left":
            # FOR YELLOW
            distMM = ((currentBlock-BlockToGoTo) * (distanceBetweenBlocksMM + blockSizeMM)) - dirOffset
            angle = -155
            pos = PICK_YELLOW
            dist = 80
        
        if direction == "right":
            # FOR RED
            distMM = ((currentBlock-BlockToGoTo) * (distanceBetweenBlocksMM + blockSizeMM)) + dirOffset
            angle = 163
            pos = PICK_RED
            dist = 100
 
       
        drive_base.straight(distMM)
        turnToAngle(targetAngle=angle, speed=300)
        setPickersPosition(pos, speed=100)
        drive_base.straight(dist)  # Reduced from 200 to 80
        # waitForRightButtonPress()
        setPickersPosition(position=ALL_UP, speed=200)
    
    yellowIndex = block_dict["yellow"]
    redIndex = block_dict["red"]

    if yellowIndex > redIndex:
        pickFirstBlock(yellowIndex, "left")
        pickSecondBlock(yellowIndex, redIndex, "right")
        finalIndex = redIndex
    else:
        pickFirstBlock(redIndex, "right")
        pickSecondBlock(redIndex, yellowIndex, "left")
        finalIndex = yellowIndex
    
    return finalIndex


def redYellowDropOffs(finalIndex):
    # Backup to the sixth block
    angle = -90
    turnToAngle(targetAngle=angle, speed=300)
    distCM = (6 - finalIndex) * 9.5
    gyroStraightWithDrive(distance=distCM, speed=800, backward=True, targetAngle=angle, slowDown=False)

    # Flush against the wall
    #flushUsingStallDetect(distance = 32, backward = True, speed = 800, max_load=120)
    gyroStraightWithDrive(distance=28,speed = 400,backward=True,targetAngle=-90)
    hub.imu.reset_heading(angle)
    wait(200)
    drive_base.straight(40)
    turnToAngle(targetAngle=0, speed=800, oneWheelTurn=True)
    
    angle = 0
    turnToAngle(targetAngle=angle, speed=800)
    gyroStraightWithDrive(distance=40, speed=800, targetAngle=angle, slowDown=True)
    # waitForRightButtonPress()
    if gyroStraightWithDrive(distance=15, speed=500, targetAngle=angle, tillBlackLine=True, slowDown=False) == False:
        print("Didn't find line infront of red yellow dropoffs")

    gyroStraightWithDrive(distance=26.5, speed=300, targetAngle=angle)
    # setPickersPosition(DROP_BLOCKS, speed=50)
    setPickersPosition(position = MIDDLE_DROP_BLOCKS, speed = 150)
    wait(700)
    setPickersPosition(position = DROP_BLOCKS, speed = 150)
    gyroStraightWithDrive(distance=3, targetAngle=angle, speed=200, backward=True)
    setPickersPosition(DROP_BLOCKS_LOWER, speed = 100)


def greenWhiteDropOffs(finalIndex):
    # Backup to the sixth block
    angle = 90
    turnToAngle(targetAngle=angle, speed=300)
    distCM = (finalIndex) * 9.5
    gyroStraightWithDrive(distance=distCM, speed=800, backward=True, targetAngle=angle, slowDown=False)

    # Flush against the wall
    #flushUsingStallDetect(distance = 28, backward = True, speed = 800)
    gyroStraightWithDrive(distance=18, speed=600, backward=True, targetAngle=angle)
    wait(200)
    hub.imu.reset_heading(angle)
    
    drive_base.straight(30)
    turnToAngle(targetAngle=0, speed=800, oneWheelTurn=True)
    
    angle = 0
    turnToAngle(targetAngle=angle, speed=800)
    gyroStraightWithDrive(distance=40, speed=800, targetAngle=angle, slowDown=True)
    # waitForRightButtonPress()
    if gyroStraightWithDrive(distance=15, speed=500, targetAngle=angle, tillBlackLine=True, slowDown=False) == False:
        print("Didn't find line infront of white green dropoffs")
    gyroStraightWithDrive(distance=26.5, speed=300, targetAngle=angle)
    # setPickersPosition(DROP_BLOCKS, speed=50)
    setPickersPosition(position = MIDDLE_DROP_BLOCKS, speed = 150)
    wait(700)
    setPickersPosition(position = DROP_BLOCKS, speed = 150)
    gyroStraightWithDrive(distance=3, targetAngle=angle, speed=200, backward=True)
    setPickersPosition(DROP_BLOCKS_LOWER, speed = 100)



def redYellowDropOffsTwoLineCatch(finalIndex):
    # Backup to the sixth block
    angle = -90
    turnToAngle(targetAngle=angle, speed=300)
    distCM = (6 - finalIndex) * 9.5
    gyroStraightWithDrive(distance=distCM, speed=800, backward=True, targetAngle=angle, slowDown=False)

    # Flush against the wall
    #flushUsingStallDetect(distance = 32, backward = True, speed = 800, max_load=120)
    gyroStraightWithDrive(distance=32,speed=800,backward=True,targetAngle=-90)
    hub.imu.reset_heading(angle)
    wait(200)
    drive_base.straight(40)
    turnToAngle(targetAngle=0, speed=800, oneWheelTurn=True)
    
    angle = 0
    turnToAngle(targetAngle=angle, speed=800)
    gyroStraightWithDrive(distance=25, speed=800, targetAngle=angle, slowDown=True)


    # waitForRightButtonPress()
    if gyroStraightWithDrive(distance=15, speed=500, targetAngle=angle, tillBlackLine=True, slowDown=False) == False:
        print("Didn't find grey line infront of red yellow dropoffs")
    
    drive_base.straight(60)
    if gyroStraightWithDrive(distance=15, speed=500, targetAngle=angle, tillBlackLine=True, slowDown=False) == False:
        print("Didn't find black line infront of red yellow dropoffs")


    gyroStraightWithDrive(distance=26.5, speed=300, targetAngle=angle)
    # setPickersPosition(DROP_BLOCKS, speed=50)
    setPickersPosition(position = MIDDLE_DROP_BLOCKS, speed = 150)
    wait(700)
    setPickersPosition(position = DROP_BLOCKS, speed = 150)
    gyroStraightWithDrive(distance=3, targetAngle=angle, speed=200, backward=True)
    setPickersPosition(DROP_BLOCKS_LOWER, speed = 100)


def greenWhiteDropOffsTwoLineCatch(finalIndex):
    # Backup to the sixth block
    angle = 90
    turnToAngle(targetAngle=angle, speed=300)
    distCM = (finalIndex) * 9.5
    gyroStraightWithDrive(distance=distCM, speed=800, backward=True, targetAngle=angle, slowDown=False)

    # Flush against the wall
    #flushUsingStallDetect(distance = 28, backward = True, speed = 800)
    gyroStraightWithDrive(distance=21, speed=600, backward=True, targetAngle=angle)
    wait(200)
    hub.imu.reset_heading(angle)
    
    drive_base.straight(30)
    turnToAngle(targetAngle=0, speed=800, oneWheelTurn=True)
    
    angle = 0
    turnToAngle(targetAngle=angle, speed=800)
    gyroStraightWithDrive(distance=40, speed=800, targetAngle=angle, slowDown=True)
    # waitForRightButtonPress()
    if gyroStraightWithDrive(distance=15, speed=500, targetAngle=angle, tillBlackLine=True, slowDown=False) == False:
        print("Didn't find line infront of white green dropoffs")
    gyroStraightWithDrive(distance=26.5, speed=300, targetAngle=angle)
    # setPickersPosition(DROP_BLOCKS, speed=50)
    setPickersPosition(position = MIDDLE_DROP_BLOCKS, speed = 150)
    wait(700)
    setPickersPosition(position = DROP_BLOCKS, speed = 150)
    gyroStraightWithDrive(distance=3, targetAngle=angle, speed=200, backward=True)
    setPickersPosition(DROP_BLOCKS_LOWER, speed = 100)



def pushDrone():
    gyroStraightWithDrive(distance=5, speed=200, backward=True, targetAngle=90)
    curve_with_accel_options(radius = 100, angle = 90, speed = 800, dont_accelerate = True)
    setPickersPosition(position=HOLD_DRONE,wait=False)
    drive_base.settings(800, 800, 800, 800)
    #gyroStraightWithDrive(distance=75, speed=800, targetAngle=180)
    drive_base.straight(750)
    setPickersPosition(position=ALL_UP,wait=False)
    
    #gyroStraightWithDrive(distance=84, speed=800, backward=True, targetAngle=180)
    drive_base.straight(-850)
    curve_with_accel_options(radius = -80, angle = -90, speed = 800, dont_accelerate = True, dont_decelerate = True)

    #flushUsingStallDetect(distance = 17, backward = True)
    gyroStraightWithDrive(distance=17,speed=600,backward=True,targetAngle=-90)
    wait(100)
    hub.imu.reset_heading(-90)
    # waitForRightButtonPress()


def pickupGreenWhite(block_dict):
    def pickFirstBlock(block, direction):
        OffsetDistanceMM = 250
        distanceBetweenBlocksMM = 63
        blockSizeMM = 32
        dirOffset = 0

        if direction == "left":
            # FOR WHITE
            distMM = (OffsetDistanceMM + ((6 - block) * (distanceBetweenBlocksMM + blockSizeMM))) - dirOffset
            angle = -155
            pos = PICK_WHITE
            dist = 80

        if direction == "right":
            # ---- RIGHT ARM IS LOWER -----
            # FOR GREEN
            distMM = (OffsetDistanceMM + ((6 - block) * (distanceBetweenBlocksMM + blockSizeMM))) + dirOffset
            angle = 161
            pos = PICK_GREEN
            dist = 100

        
        drive_base.straight(distMM)
        #gyroStraightWithDrive(distance=distMM/10,speed=300,targetAngle=-90)
        setPickersPosition(pos)
        turnToAngle(targetAngle=angle, speed=300)
        drive_base.straight(dist)  # Reduced from 200 to 80
        # waitForRightButtonPress()
        setPickersPosition(position=ALL_UP, speed=300)
        drive_base.straight(-90)
        turnToAngle(targetAngle=-90, speed=300)

    def pickSecondBlock(currentBlock, BlockToGoTo, direction):
        distanceBetweenBlocksMM = 63
        blockSizeMM = 32
        dirOffset = 0

        if direction == "left":
            # FOR WHITE
            distMM = ((currentBlock-BlockToGoTo) * (distanceBetweenBlocksMM + blockSizeMM)) - dirOffset
            angle = -155
            pos = PICK_WHITE
            dist = 80

        if direction == "right":
            # FOR GREEN
            distMM = ((currentBlock-BlockToGoTo) * (distanceBetweenBlocksMM + blockSizeMM)) + dirOffset
            angle = 163
            pos = PICK_GREEN
            dist = 100

        
        drive_base.straight(distMM)
        turnToAngle(targetAngle=angle, speed=200)
        setPickersPosition(pos, speed=100)
        drive_base.straight(dist)  # Reduced from 200 to 80
        # waitForRightButtonPress()
        setPickersPosition(position=ALL_UP, speed=200)

    angle = 0
    # gyroStraightWithDrive(distance= 76, speed=800, backward=True, targetAngle=angle)
    drive_base.settings(800, 800, 800, 800)
    drive_base.straight(-630)
    curve_with_accel_options(radius = -100, angle = 90, speed = 800, dont_accelerate = True, dont_decelerate = True)
    # flushUsingStallDetect(distance = 10, backward = True, speed = 800)
    gyroStraightWithDrive(distance = 13, speed = 800, backward = True, targetAngle = -90)
    wait(100)
    hub.imu.reset_heading(-90)

    whiteIndex = block_dict["white"]
    greenIndex = block_dict["green"]

    if whiteIndex > greenIndex:
        pickFirstBlock(whiteIndex, "left")
        pickSecondBlock(whiteIndex, greenIndex, "right")
        finalIndex = greenIndex
    else:
        pickFirstBlock(greenIndex, "right")
        pickSecondBlock(greenIndex, whiteIndex, "left")
        finalIndex = whiteIndex

    return finalIndex


def park():
    angle = 0
    gyroStraightWithDrive(distance=60, speed=700, backward=True, targetAngle=angle)

    # Flushing against the wall
    angle = 90
    turnToAngle(targetAngle=angle, speed=300)
    gyroStraightWithDrive(distance=12, speed=350, backward=True, targetAngle=angle)
    wait(100)
    gyroStraightWithDrive(distance=45, speed=500, targetAngle=angle)
    if gyroStraightWithDrive(distance=20, speed=300, targetAngle=angle, tillBlackLine=True) == False:
        print("The black line catch before park missed")
    wait(200)
    gyroStraightWithDrive(distance=9, speed=150, targetAngle=angle, backward=True)
    setBallPickerPosition(DROP_BALLS, wait=False)
    setPickersPosition(ALL_UP,wait=False)

    angle=0
    turnToAngle(targetAngle=angle, speed=300)
    #waitForRightButtonPress()
    gyroStraightWithDrive(distance=81, speed=600, targetAngle=angle,tillWhiteLine=True)
    drive_base.straight(15)
    turnToAngle(targetAngle=90, speed=200, oneWheelTurn=True, forceTurn=FORCETURN_RIGHT)

def parkevenfaster():
    angle = 0
    gyroStraightWithDrive(distance=30, speed=700, backward=True, targetAngle=angle,slowDown=False)
    setPickersPosition(ALL_UP,wait=False)
    curve(radius = -165,angle = 180,speed=800)
    #waitForRightButtonPress()
    angle = -175
    if(gyroStraightWithDrive(distance=47, speed=600, targetAngle=angle,tillWhiteLine=True,backward=True,slowDown=False)==False):
        print("Parking: Missed white line catch before parking")
    gyroStraightWithDrive(distance=5, speed=600, targetAngle=195,backward=True)

def main():
  
    ''' #Open this for individual run with timings.
    hub.imu.reset_heading(180)
    runWithTiming(resetPickers,"resetPickers")
    runWithTiming(resetBallPicker,"resetBallPicker")
    runWithTiming(goFromHomeToBallPit,"goFromHomeToBallPit")
    
    runWithTiming(getBallsFromBallPit,"getBallsFromBallPit")
    runWithTiming(dropBalls,"dropBalls")
    runWithTiming(doRoverAfterBalls,"doRoverAfterBalls")
    runWithTiming(goToBlocks,"gotoBlocks")
    block_dict = runWithTiming(detectBlocksAndDecideWhichOneToStartPickupFrom, "detectBlocks")
    runWithTiming(pushDrone, "pushDrone")
    finalIndex = runWithTimingAndStartBlockParam(pickupRedYellow, "pickupRedYellow", block_dict)
    runWithTimingAndStartBlockParam(redYellowDropOffs, "redYellowDropOff", finalIndex)
    finalIndex = runWithTimingAndStartBlockParam(pickupGreenWhite, "pickupGreenWhite", block_dict)
    runWithTimingAndStartBlockParam(greenWhiteDropOffs, "greenWhiteDropOff", finalIndex)
    runWithTiming(park, "park")
    '''

    hub.imu.reset_heading(180)
    resetPickers()
    resetBallPicker()
    goFromHomeToBallPit()
    
    getBallsFromBallPit()
    dropBallsAxleBucket()
    doRoverAfterBalls()
    goToBlocks()
    block_dict = detectBlocksAndDecideWhichOneToStartPickupFrom()
    pushDrone()
    finalIndex = pickupRedYellow( block_dict)
    redYellowDropOffsTwoLineCatch(finalIndex)
    finalIndex = pickupGreenWhite(block_dict)
    greenWhiteDropOffs(finalIndex)
    
    parkevenfaster()


def create_path_from_main():
    """
    Create a path planner instance and populate it with waypoints from the main function.
    This allows you to edit the waypoints and generate new code.
    """
    planner = PathPlanner()
    
    # Start at origin
    planner.reset_position(0, 0, 180)  # Starting heading from main()
    
    # Add waypoints based on the main function logic
    # This is a simplified version - you can add more detailed waypoints
    
    # goFromHomeToBallPit
    planner.drive_straight(13, speed=900, backward=True, target_angle=180, slow_down=False)
    planner.curve(-165, 180, speed=400)
    planner.drive_straight(3, speed=200, backward=True, detect_stall=True)
    
    # getBallsFromBallPit
    planner.set_pickers_position('PICK_GREEN', speed=300, wait=False)
    planner.drive_straight(8, speed=200, target_angle=0)
    planner.set_ball_picker_position('PICK_BALLS', speed=300, wait=True)
    planner.wait(50)
    planner.drive_straight(130, speed=400, backward=True)
    planner.drive_straight(50, speed=400)
    planner.set_ball_picker_position('SLIDE_BALL', speed=100, wait=False)
    planner.drive_straight(50, speed=400)
    planner.set_ball_picker_position('PICK_BALLS', speed=150, wait=False)
    planner.turn_to_angle(0, speed=100)
    planner.drive_straight(12, speed=100, backward=True, target_angle=0)
    planner.wait(100)
    planner.set_pickers_position('ALL_UP', wait=False)
    planner.drive_straight(5, speed=250, target_angle=0)
    planner.wait(100)
    
    # dropBallsAxleBucket
    planner.curve(110, -90, speed=400, dont_decelerate=True)
    planner.wait(100)
    planner.drive_straight(205, speed=800)
    planner.curve(110, -90, speed=800, dont_accelerate=True, dont_decelerate=True)
    planner.wait(100)
    planner.drive_straight(52, speed=200)
    planner.set_pickers_position('HALF_LIFT_LID', speed=500, wait=True)
    planner.drive_straight(4, speed=600, target_angle=180, slow_down=False)
    planner.set_pickers_position('LIFT_LID', speed=1000, wait=False)
    planner.drive_straight(6, speed=600, target_angle=180, slow_down=False)
    planner.set_ball_picker_position('DROP_BALLS', speed=200, wait=True)
    
    # doRoverAfterBalls
    planner.drive_straight(240, speed=800, backward=True)
    planner.set_ball_picker_position('ROVER', speed=600, wait=False)
    planner.curve(-205, -90, speed=850, dont_accelerate=True, dont_decelerate=True)
    
    # goToBlocks
    planner.drive_straight(150, speed=800)
    planner.set_ball_picker_position('DROP_BALLS', wait=False)
    planner.curve(120, 90, speed=800, dont_accelerate=True, dont_decelerate=True)
    planner.set_pickers_position('ALL_UP', wait=False)
    planner.turn_to_angle(0, speed=400)
    planner.drive_straight(35, speed=500, target_angle=0, till_black_line=True, slow_down=False)
    planner.drive_straight(130, speed=800)
    planner.turn_to_angle(90, speed=400)
    planner.drive_straight(10, speed=600, backward=True, target_angle=90)
    planner.wait(100)
    
    # Add some example waypoints for the rest of the path
    # (You can add more detailed waypoints based on your specific needs)
    planner.drive_straight(50, speed=400)
    planner.turn_to_angle(180, speed=400)
    planner.drive_straight(30, speed=400)
    
    return planner


def interactive_path_editor():
    """
    Launch the interactive path editor with waypoints from the main function.
    """
    print("Creating path from main function...")
    planner = create_path_from_main()
    
    print("Launching interactive waypoint editor...")
    print("You can now edit waypoints, add new ones, and generate updated code.")
    planner.interactive_waypoint_editor()
    

# Initialize the robot and set the drive base settings
initializeAndWaitForRobotReady()
# testcolordetection()  

# Uncomment one of the following lines to run:

# Run the main robot program with timing
#runWithTiming(main, "BOBAAAA🔥🔥🧋🧋:")

# Run the main robot program without timing
#main()

# Launch the interactive path editor (NEW FEATURE!)
# This allows you to edit waypoints and generate new code
interactive_path_editor()
"""
    if medianHue == 0 and medianSat == 0 and avgVal == 0:
        return "empty"
    if medianSat < 40:
        return "white"
    elif (medianHue > 320 or medianHue < 40) and medianSat > 50:
        return "red"
    elif 90 <= medianHue <= 140 and medianSat > 50:
        return "green"
    elif 30 <= medianHue <= 90 and medianSat > 50:
        return "yellow"
    else:
        return "empty"

"""

'''
# Before reshma changes
 if avgValue == 0:
            pass
        elif cleanAvgSaturation < 40:
            blockDict["white"] = currentBlockNumber - 1
        elif (medianHue > 240 or medianHue < 30) and medianSaturation > 50:
            blockDict["red"] = currentBlockNumber - 1
        elif 130 <= medianHue <= 190 and medianSaturation > 50:
            blockDict["green"] = currentBlockNumber - 1
        elif 30 <= medianHue <= 90 and medianSaturation > 50:
            blockDict["yellow"] = currentBlockNumber - 1
'''