import csv, math
from time import *
from turtle import Turtle, Screen
import os
from tkinter import *
import tkinter as tk
from PIL import Image

robot = None

# SIMULATOR CONSTANTS
REFRESH_DELAY = 3
#MAT_WIDTH_PIXELS = 2608
#MAT_HEIGHT_PIXELS = 1268
#MAT_WIDTH_CM = 237
#MAT_HEIGHT_CM = 114
#GAME_MAT = "game_mat.png"

#GAME_MAT = "small_game_mat.png"
#GAME_MAT = "large_game_mat.png"
GAME_MAT = "wro2025_junior_field.png"
MAT_WIDTH_PIXELS = 2234
MAT_HEIGHT_PIXELS = 1082
MAT_WIDTH_CM = 236.2
MAT_HEIGHT_CM = 114.3

# ROBOT CONSTANTS
ROBOT_DISTANCE_TO_FRONT = 1
ROBOT_DISTANCE_BACK = 13.5
#ROBOT_DISTANCE_BACK = 11.5 THIS IS WITHOUT THE SEPRATORS 
ROBOT_DISTANCE_TO_BACK_WITH_BUCKET_DOWN = 28
ROBOT_WIDTH_WITH_BUCKET_DOWN = 23.5
ROBOT_WIDTH =16

ROBOT_DISTANCE_TO_FRONT_WITH_BUCKET_DOWN = 7.5
   
PICK_BLOCKS = 0
DROP_BLOCKS = 1
DROP_BLOCKS2 = 2
DROP_BLOCKS_LOWER = 3
BELOW_LID = 4
HALF_LIFT_LID = 5
LIFT_LID = 6
ALL_UP = 7
PICK_BALLS = 8
SLIDE_BALL = 9
DROP_BALLS = 10
HOLD_DRONE = 11
PICK_YELLOW = 12
PICK_RED = 12
PICK_GREEN = 12
PICK_WHITE = 12
GROUND = 14
MIDDLE_DROP_BLOCKS = 15


FRONT_BUCKET_DOWN = 0
FRONT_BUCKET_UP = 1
BACK_BUCKET_DOWN = 0
BACK_BUCKET_UP = 1

STRUCTURE1_X =  21
STRUCTURE1_Y = 46

STRUCTURE2_X = 17
STRUCTURE2_Y = 90

STRUCTURE1_WIDTH = 10
STRUCTURE1_HEIGHT = 10

STRUCTURE2_WIDTH = 10
STRUCTURE2_HEIGHT = 10

FLUSH_OFFSET = 2  # cm away from the structure

# Each line is a dict with type ('vertical' or 'horizontal') and position (in cm)
BLACK_LINES = [
    # Vertical lines (x positions)
    {"type": "vertical", "x": 18},
    {"type": "vertical", "x": 32},
    {"type": "vertical", "x": 118},
    {"type": "vertical", "x": 204},
    {"type": "vertical", "x": 218},

    # Horizontal lines (y positions)
    {"type": "horizontal", "y": 29},
    {"type": "horizontal", "y": 44},
    {"type": "horizontal", "y": 57},
    {"type": "horizontal", "y": 85},
    {"type": "horizontal", "y": 100},
]

PICK_GREEN = 0
def setPickersPosition(position, speed=500, wait = True): 
    if robot is not None:
        robot.positionFrontBucket(position)

ROVER = 0
def setBallPickerPosition(position, speed=500, wait = True): 
    if robot is not None:
        robot.positionBackBucket(position)

def resetPickers():
    if robot is not None:
        robot.positionFrontBucket(FRONT_BUCKET_DOWN)
def resetBallPicker(): 
    if robot is not None:
        robot.positionBackBucket(BACK_BUCKET_UP)

def waitForRightButtonPress(): pass
def waitForLeftButtonPress(): pass
def getDriveBaseSettings(): return (200, 200, 100, 100)
def setDriveBaseSettings(speed, accel, turn_rate, turn_accel): pass
def medianOfAList(lst): return sorted(lst)[len(lst)//2] if lst else 0

# Dummy drive_base and hub objects for simulation
class DummyDriveBase:
    def straight(self, dist):
        if robot is not None:
            gyroStraightWithDrive(distance=dist/10, speed = 300, targetAngle=robot.getAngle())
    def stop(self): pass
    def settings(self, a, b, c,d ): 
        if robot is not None:
            print(robot.angle)

    def turn(self, angle):
        turnToAngle(targetAngle = angle, speed = 400)

class DummyIMU:
    def reset_heading(self, angle): 
        if robot is not None:
            robot.setAngle(angle)

class DummyHub:
    imu = DummyIMU()

drive_base = DummyDriveBase()
hub = DummyHub()

# Simulated block color sensor
class DummyBlockColor:
    def hsv(self):
        from collections import namedtuple
        HSV = namedtuple('HSV', ['h', 's', 'v'])
        return HSV(h=50, s=100, v=100)  # Example values

block_color = DummyBlockColor()

# Line follower constants.
LINE_FOLLOWER_EDGE_LEFT = 0
LINE_FOLLOWER_EDGE_RIGHT = 1

class Stop:
    BRAKE = 0
    HOLD = 1
    COAST = 2

class front_bucket:
    def hold():
        return

class ball_picker:
    def run_target(speed =  500, target_angle = 0, wait=True):
        return
    
FORCETURN_NONE = 1

screen = Screen()
turtle = Turtle()
screen2 = Screen()

# dTurtle is the turtle used for drawing the distance measuring tool line
dturtle = Turtle()
button = None
dataButton = None

startPosX = -100
startPosY = -100
DISTANCE = 0
HORIZONTAL_LINE = 1
VERTICAL_LINE = 2

MeasureAction = DISTANCE

# NOTE THIS SHOULD BE ON ALWAYS. We only use this as FALSE
# when we are testing the simulator_utils for some internal functionality.
DRAW_ROBOT = True

# These functions are redefined later with proper scaling
# Removing duplicates to avoid conflicts

# Robot class
class Robot():
    width = None
    distance_to_front = None
    distance_to_back = None
    distance_to_back_with_bucket_down = None
    distance_to_front_cm = ROBOT_DISTANCE_TO_FRONT
    distance_to_back_cm = ROBOT_DISTANCE_BACK
    distance_to_back_with_bucket_down_cm = ROBOT_DISTANCE_TO_BACK_WITH_BUCKET_DOWN

    def convertTo360(self,angle):
        retAngle = angle * -1
        if (retAngle < 0):
            return 360 + retAngle
        else:
            return retAngle
          
    def __init__(self, initial_angle = 0, back_bucket_position=BACK_BUCKET_DOWN, front_bucket_position=FRONT_BUCKET_DOWN,
                 center_x = 23.5, center_y = 21.5):
        # Always update these in case SCALE has changed
        Robot.width = convertXCMToPixels(ROBOT_WIDTH)
        Robot.distance_to_front = convertYCMToPixels(ROBOT_DISTANCE_TO_FRONT)
        Robot.distance_to_back = convertYCMToPixels(ROBOT_DISTANCE_BACK)
        Robot.distance_to_back_with_bucket_down = convertYCMToPixels(ROBOT_DISTANCE_TO_BACK_WITH_BUCKET_DOWN)
        
        self.angle = initial_angle
        self.center_x = center_x
        self.center_y = center_y
        self.back_bucket_position = back_bucket_position
        if back_bucket_position == BACK_BUCKET_DOWN:
            self.back_bucket_down = True
        else:
            self.back_bucket_down = False
        self.front_bucket_position = front_bucket_position
        self.draw()

    def getPosition(self):
        return self.angle, self.center_x, self.center_y

    def getAngle(self):
        return self.angle

    def setAngle(self, angle):
        self.angle = angle
        self.draw()

    def resetPosition(self, x, y, angle):
        self.angle = angle
        self.center_x = x
        self.center_y = y
        self.draw()

    def turnToAngle(self, targetAngle, speed):
        self.angle = targetAngle
        self.draw()

    def flushUsingStallDetect(self, distance, max_load = 100, speed=200, backward=False, acceleration=500):
        DIST = 20

        # --- Structure 1 (Ball Pit) ---
        if abs(self.center_y - STRUCTURE1_Y) < DIST:
            if self.angle == 0:
                if not backward:
                    print("Flushing at Structure 1, heading 0, forward")
                    self.center_x = STRUCTURE1_X + STRUCTURE1_WIDTH / 2 + FLUSH_OFFSET - Robot.distance_to_front_cm
                else:
                    print("Flushing at Structure 1, heading 0, reverse")
                    if self.back_bucket_down:
                        self.center_x = STRUCTURE1_X + STRUCTURE1_WIDTH / 2 + FLUSH_OFFSET + Robot.distance_to_back_with_bucket_down_cm
                    else:
                        self.center_x = STRUCTURE1_X + STRUCTURE1_WIDTH / 2 + FLUSH_OFFSET + Robot.distance_to_back_cm
                self.center_y = STRUCTURE1_Y
                self.draw()
                return
            if self.angle == 180 or self.angle == -180:
                if not backward:
                    print("Flushing at Structure 1, heading 180, forward")
                    self.center_x = STRUCTURE1_X - STRUCTURE1_WIDTH / 2 - FLUSH_OFFSET + Robot.distance_to_front_cm
                else:
                    print("Flushing at Structure 1, heading 180, reverse")
                    if self.back_bucket_down:
                        self.center_x = STRUCTURE1_X + STRUCTURE1_WIDTH / 2 + FLUSH_OFFSET - Robot.distance_to_back_with_bucket_down_cm
                    else:
                        self.center_x = STRUCTURE1_X + STRUCTURE1_WIDTH / 2 + FLUSH_OFFSET - Robot.distance_to_back_cm
                self.center_y = STRUCTURE1_Y
                self.draw()
                return

        if abs(self.center_x - STRUCTURE1_X) < DIST:
            if self.angle == 90:
                if not backward:
                    print("Flushing at Structure 1, heading 90, forward")
                    self.center_y = STRUCTURE1_Y + STRUCTURE1_HEIGHT / 2 + FLUSH_OFFSET - Robot.distance_to_front_cm
                else:
                    print("Flushing at Structure 1, heading 90, reverse")
                    if self.back_bucket_down:
                        self.center_y = STRUCTURE1_Y - STRUCTURE1_HEIGHT / 2 - FLUSH_OFFSET + Robot.distance_to_back_with_bucket_down_cm
                    else:
                        self.center_y = STRUCTURE1_Y - STRUCTURE1_HEIGHT / 2 - FLUSH_OFFSET + Robot.distance_to_back_cm
                self.center_x = STRUCTURE1_X
                self.draw()
                return
            if self.angle == -90:
                if not backward:
                    print("Flushing at Structure 1, heading -90, forward")
                    self.center_y = STRUCTURE1_Y - STRUCTURE1_HEIGHT / 2 - FLUSH_OFFSET + Robot.distance_to_front_cm
                else:
                    print("Flushing at Structure 1, heading -90, reverse")
                    if self.back_bucket_down:
                        self.center_y = STRUCTURE1_Y + STRUCTURE1_HEIGHT / 2 + FLUSH_OFFSET - Robot.distance_to_back_with_bucket_down_cm
                    else:
                        self.center_y = STRUCTURE1_Y + STRUCTURE1_HEIGHT / 2 + FLUSH_OFFSET - Robot.distance_to_back_cm
                self.center_x = STRUCTURE1_X
                self.draw()
                return

        # --- Structure 2 (Blocks Drop) ---
        if abs(self.center_y - STRUCTURE2_Y) < DIST:
            if self.angle == 0:
                if not backward:
                    print("Flushing at Structure 2, heading 0, forward")
                    self.center_x = STRUCTURE2_X + STRUCTURE2_WIDTH / 2 + FLUSH_OFFSET - Robot.distance_to_front_cm
                else:
                    print("Flushing at Structure 2, heading 0, reverse")
                    if self.back_bucket_down:
                        self.center_x = STRUCTURE2_X - STRUCTURE2_WIDTH / 2 - FLUSH_OFFSET + Robot.distance_to_back_with_bucket_down_cm
                    else:
                        self.center_x = STRUCTURE2_X - STRUCTURE2_WIDTH / 2 - FLUSH_OFFSET + Robot.distance_to_back_cm
                self.center_y = STRUCTURE2_Y
                self.draw()
                return
            if self.angle == 180 or self.angle == -180:
                if not backward:
                    print("Flushing at Structure 2, heading 180, forward")
                    self.center_x = STRUCTURE2_X + STRUCTURE2_WIDTH / + FLUSH_OFFSET + 7.5
                else:
                    print("Flushing at Structure 2, heading 180, reverse")
                    if self.back_bucket_down:
                        self.center_x = STRUCTURE2_X + STRUCTURE2_WIDTH / 2 + FLUSH_OFFSET - Robot.distance_to_back_with_bucket_down_cm
                    else:
                        self.center_x = STRUCTURE2_X + STRUCTURE2_WIDTH / 2 + FLUSH_OFFSET - Robot.distance_to_back_cm
                self.center_y = STRUCTURE2_Y
                self.draw()
                return

        if abs(self.center_x - STRUCTURE2_X) < DIST:
            if self.angle == 90:
                if not backward:
                    print("Flushing at Structure 2, heading 90, forward")
                    self.center_y = STRUCTURE2_Y + STRUCTURE2_HEIGHT / 2 + FLUSH_OFFSET - Robot.distance_to_front_cm
                else:
                    print("Flushing at Structure 2, heading 90, reverse")
                    if self.back_bucket_down:
                        self.center_y = STRUCTURE2_Y - STRUCTURE2_HEIGHT / 2 - FLUSH_OFFSET + Robot.distance_to_back_with_bucket_down_cm
                    else:
                        self.center_y = STRUCTURE2_Y - STRUCTURE2_HEIGHT / 2 - FLUSH_OFFSET + Robot.distance_to_back_cm
                self.center_x = STRUCTURE2_X
                self.draw()
                return
            if self.angle == -90:
                if not backward:
                    print("Flushing at Structure 2, heading -90, forward")
                    self.center_y = STRUCTURE2_Y - STRUCTURE2_HEIGHT / 2 - FLUSH_OFFSET + Robot.distance_to_front_cm
                else:
                    print("Flushing at Structure 2, heading -90, reverse")
                    if self.back_bucket_down:
                        self.center_y = STRUCTURE2_Y + STRUCTURE2_HEIGHT / 2 + FLUSH_OFFSET - Robot.distance_to_back_with_bucket_down_cm
                    else:
                        self.center_y = STRUCTURE2_Y + STRUCTURE2_HEIGHT / 2 + FLUSH_OFFSET - Robot.distance_to_back_cm
                self.center_x = STRUCTURE2_X
                self.draw()
                return

        if (self.angle == 0):
            if backward == False:
                self.resetPosition(MAT_WIDTH_CM - Robot.distance_to_front_cm, self.center_y, self.angle)
            if backward == True:
                if (self.back_bucket_down):
                    self.resetPosition(Robot.distance_to_back_with_bucket_down_cm, self.center_y, self.angle)
                else:
                    self.resetPosition(Robot.distance_to_back_cm, self.center_y, self.angle)
        elif (self.angle == 180): 
            if backward == True:
                if (self.back_bucket_down):
                    self.resetPosition(MAT_WIDTH_CM - Robot.distance_to_back_with_bucket_down_cm, self.center_y, self.angle)
                else:
                    self.resetPosition(MAT_WIDTH_CM - Robot.distance_to_back_cm, self.center_y, self.angle)
            if backward == False:
                self.resetPosition(Robot.distance_to_front_cm, self.center_y, self.angle)
        elif (self.angle == -90): 
            if backward == True:
                if (self.back_bucket_down):
                    self.resetPosition(self.center_x, Robot.distance_to_back_with_bucket_down_cm, self.angle)
                else:
                    self.resetPosition(self.center_x, Robot.distance_to_back_cm, self.angle)
            if backward == False:
                self.resetPosition(self.center_x, MAT_HEIGHT_CM-Robot.distance_to_front_cm, self.angle)
        elif (self.angle == 90):             
            if backward == False:
                self.resetPosition(self.center_x, Robot.distance_to_front_cm, self.angle)
            if backward == True:
                if (self.back_bucket_down):
                    self.resetPosition(self.center_x, MAT_HEIGHT_CM-Robot.distance_to_back_with_bucket_down_cm, self.angle)
                else:
                    self.resetPosition(self.center_x, MAT_HEIGHT_CM-Robot.distance_to_back_cm, self.angle)
        self.draw()
        
    # Check if the gyroStraight is going to go past any wall.
    # If it is, then return the distance that puts it at the wall.
    def _getDistanceCheckingForDrivePastWall(self, distance, targetAngle, backward, optional_simulator_max_distance_from_wall):
        if (optional_simulator_max_distance_from_wall != None):
            if backward == False:
                if (distance > (optional_simulator_max_distance_from_wall - Robot.distance_to_front_cm)):
                    distance = optional_simulator_max_distance_from_wall - Robot.distance_to_front_cm
            else:
                if self.back_bucket_down == True:
                    if (distance > (optional_simulator_max_distance_from_wall - Robot.distance_to_back_with_bucket_down_cm)):
                        distance = optional_simulator_max_distance_from_wall - Robot.distance_to_back_with_bucket_down_cm
                else:    
                    if (distance > (optional_simulator_max_distance_from_wall - Robot.distance_to_back_cm)):
                        distance = optional_simulator_max_distance_from_wall - Robot.distance_to_back_cm

            return distance
        
        if (targetAngle == 90 or targetAngle == 450):
            if(backward == False):
                if (distance > self.center_y - Robot.distance_to_front_cm): 
                    distance = self.center_y - Robot.distance_to_front_cm
            else:
                if (self.back_bucket_down == True):
                    if (distance > (MAT_HEIGHT_CM - self.center_y - Robot.distance_to_back_with_bucket_down_cm)):
                        distance = (MAT_HEIGHT_CM - self.center_y - Robot.distance_to_back_with_bucket_down_cm)
                else:
                    if (distance > (MAT_HEIGHT_CM - self.center_y - Robot.distance_to_back_cm)):
                        distance = (MAT_HEIGHT_CM - self.center_y - Robot.distance_to_back_cm)                
        elif (targetAngle == -90 or targetAngle == 270):
            if(backward == True):
                if (self.back_bucket_down == True):
                    if (distance > self.center_y - Robot.distance_to_back_with_bucket_down_cm): 
                        distance = self.center_y - Robot.distance_to_back_with_bucket_down_cm
                else:
                    if (distance > self.center_y - Robot.distance_to_back_cm): 
                        distance = self.center_y - Robot.distance_to_back_cm
            else:
                if (distance > (MAT_HEIGHT_CM - self.center_y - Robot.distance_to_front_cm)):
                        distance = (MAT_HEIGHT_CM - self.center_y - Robot.distance_to_front_cm)
        elif (targetAngle == -180 or targetAngle == 180):
            if(backward == False):
                if (distance > self.center_x - Robot.distance_to_front_cm): 
                    distance = self.center_x - Robot.distance_to_front_cm
            else:
                if (self.back_bucket_down == True):
                    if (distance > (MAT_WIDTH_CM - self.center_x - Robot.distance_to_back_with_bucket_down_cm)):
                        distance = (MAT_WIDTH_CM - self.center_x - Robot.distance_to_back_with_bucket_down_cm)
                else:
                    if (distance > (MAT_WIDTH_CM - self.center_x - Robot.distance_to_back_cm)):
                        distance = (MAT_WIDTH_CM - self.center_x - Robot.distance_to_back_cm)                
        elif (targetAngle == 0 or targetAngle == 360):
            if(backward == True):
                if (self.back_bucket_down == True):
                    if (distance > (self.center_x - Robot.distance_to_back_with_bucket_down_cm)):
                        distance = (self.center_x - Robot.distance_to_back_with_bucket_down_cm)
                else:
                    if (distance > (self.center_x - Robot.distance_to_back_cm)):
                        distance = (self.center_x - Robot.distance_to_back_cm)                
            else:
                if (distance > (MAT_WIDTH_CM - self.center_x - Robot.distance_to_front_cm)):
                    distance = (MAT_WIDTH_CM - self.center_x - Robot.distance_to_front_cm)

        return distance
        
    def gyroStraightWithDrive(self, distance, targetAngle, speed, backward, tillBlackLine, optional_simulator_max_distance_from_wall, flushForSimulator=False):
        self.angle = targetAngle

        # If flushForSimulator is True, check for structure flush
        if flushForSimulator:
            # Try to flush using the same logic as flushUsingStallDetect
            # (You may want to pass the same parameters as flushUsingStallDetect, or just use defaults)
            self.flushUsingStallDetect(distance, speed=speed, backward=backward)
            return

        def get_next_vertical_line(current_x, direction):
            candidates = [line["x"] for line in BLACK_LINES if line["type"] == "vertical"]
            if direction > 0:
                right_lines = [x for x in candidates if x > current_x]
                return min(right_lines) if right_lines else None
            else:
                left_lines = [x for x in candidates if x < current_x]
                return max(left_lines) if left_lines else None

        def get_next_horizontal_line(current_y, direction):
            candidates = [line["y"] for line in BLACK_LINES if line["type"] == "horizontal"]
            if direction > 0:
                up_lines = [y for y in candidates if y > current_y]
                return min(up_lines) if up_lines else None
            else:
                down_lines = [y for y in candidates if y < current_y]
                return max(down_lines) if down_lines else None

        if tillBlackLine:
            if targetAngle in [90, -90]:
                direction = 1 if (targetAngle == 90) ^ backward else -1
                next_line = get_next_horizontal_line(self.center_y, direction)
                if next_line is not None:
                    distance = abs(next_line - self.center_y)
            elif targetAngle in [0, 180, -180, 360]:
                direction = 1 if ((targetAngle == 0 or targetAngle == 360) ^ backward) else -1
                next_line = get_next_vertical_line(self.center_x, direction)
                if next_line is not None:
                    distance = abs(next_line - self.center_x)
                    
        distance = self._getDistanceCheckingForDrivePastWall(distance, targetAngle, backward, optional_simulator_max_distance_from_wall)
        
        if backward:
            self.center_x = self.center_x - (distance * math.cos(math.radians(self.angle)))
            self.center_y = self.center_y + (distance * math.sin(math.radians(self.angle)))
        else:
            self.center_x = self.center_x + (distance * math.cos(math.radians(self.angle)))
            self.center_y = self.center_y - (distance * math.sin(math.radians(self.angle)))

        self.draw()

    def followBlackLinePID(self, distanceInMM, speed, edge, controlColor = 63, 
                       kp=0.5, ki=0, kd=0, correctionBasedSpeed = False, slowStart = True, 
                       slowDown = True, printDebugMessages=False,
                       additionalFunctionToCall = None,
                       slowSpeedOverride = 100):
        distance = distanceInMM / 10
        self.center_x = self.center_x + (distance * math.cos(math.radians(self.angle)))
        self.center_y = self.center_y - (distance * math.sin(math.radians(self.angle)))
        self.draw()

    def _rotateCoordinatePlane(self, rotate_by):
        self.center_x = (self.center_x * math.cos(math.radians(rotate_by))) + (self.center_y * math.sin(math.radians(rotate_by)))
        self.center_y = (self.center_y * math.cos(math.radians(rotate_by))) - (self.center_x * math.sin(math.radians(rotate_by)))
        self.angle = self.angle + rotate_by

    def _curve_internal(self, radius=25, angle=90, speed=600, acceleration=200, optional_simulator_end_point = None, 
                        optional_simulator_end_angle = None):
        radius = radius / 10

        if (optional_simulator_end_angle != None and optional_simulator_end_point != None):
            self.angle = optional_simulator_end_angle
            self.center_x = optional_simulator_end_point[0]
            self.center_y = optional_simulator_end_point[1]
            self.draw()
            return

        current_angle = self.angle
        if (self.angle % 90 != 0):
            current_angle = round(self.angle / 90) * 90
    
        def _robotPointing90NegativeRadiusPositiveAnglePositive():
            self.center_x = self.center_x + (radius * (1 - math.cos(math.radians(angle))))
            self.center_y = self.center_y + (radius * math.sin(math.radians(angle)))
            self.angle = self.angle + angle
            if (self.angle > 180):
                self.angle = self.angle - 360

            '''    
            # CurrentAngle=-90, Radius=+ve, angle=+ve, 
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x + (radius / 2)
                self.center_y = self.center_y + (radius / 2)
                self.angle = -45
            else:
                self.center_x = self.center_x + radius
                self.center_y = self.center_y + radius
                self.angle = 0
            '''

        def _robotPointing90NegativeRadiusPositiveAngleNegative():
            self.center_x = self.center_x - (radius*(1 - math.cos(math.radians(-1 * angle))))
            self.center_y = self.center_y + (radius * math.sin(math.radians(-1 * angle)))
            self.angle = self.angle + angle
            if (self.angle < -180):
                self.angle = 360 + self.angle

            '''
            # CurrentAngle=-90, Radius=+ve, angle=-ve, 
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x - (radius / 2)
                self.center_y = self.center_y + (radius / 2)
                self.angle = -135
            else:
                self.center_x = self.center_x - radius
                self.center_y = self.center_y + radius
                self.angle = -180
            '''

        def _robotPointing90NegativeRadiusNegativeAnglePositive():
            self.center_x = self.center_x + (-1 * radius*(1 - math.cos(math.radians(angle))))
            self.center_y = self.center_y - (-1 * radius * math.sin(math.radians(angle)))
            self.angle = self.angle - angle
            if (self.angle < -180):
                self.angle = 360 + self.angle

            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x - (radius / 2)
                self.center_y = self.center_y + (radius / 2)
                self.angle = -135
            else:
                self.center_x = self.center_x - radius
                self.center_y = self.center_y + radius
                self.angle = -180
            '''

        def _robotPointing90NegativeRadiusNegativeAngleNegative():
            self.center_x = self.center_x - (-1 * radius*(1 - math.cos(math.radians(-1 * angle))))
            self.center_y = self.center_y - (-1 * radius * math.sin(math.radians(-1 * angle)))
            self.angle = self.angle - angle
            if (self.angle > 180):
                self.angle = self.angle - 360

            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x + (radius / 2)
                self.center_y = self.center_y + (radius / 2)
                self.angle = -45
            else:
                self.center_x = self.center_x + radius
                self.center_y = self.center_y + radius
                self.angle = 0
            '''

        def _robotPointingZeroRadiusPositiveAnglePositive():
            self.center_x = self.center_x + (radius * math.sin(math.radians(angle)))
            self.center_y = self.center_y - (radius * (1 - math.cos(math.radians(angle))))
            self.angle = angle

            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x + (radius/2)
                self.center_y = self.center_y - (radius/2)
                self.angle = 45
            else:   
                self.center_x = self.center_x + radius
                self.center_y = self.center_y - radius
                self.angle = 90
            '''

        def _robotPointingZeroRadiusPositiveAngleNegative():
            self.center_x = self.center_x + (radius * math.sin(math.radians(abs(angle))))
            self.center_y = self.center_y + (radius * (1 - math.cos(math.radians(abs(angle)))))
            self.angle = angle
            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x + (radius/2)
                self.center_y = self.center_y + (radius/2)
                self.angle = -45
            else:
                self.center_x = self.center_x + radius
                self.center_y = self.center_y + radius
                self.angle = -90
            '''

        def _robotPointingZeroRadiusNegativeAnglePositive():
            self.center_x = self.center_x - (-1 * radius * math.sin(math.radians(abs(angle))))
            self.center_y = self.center_y - (-1 * radius * (1 - math.cos(math.radians(abs(angle)))))
            self.angle = -1 * angle
            
            '''
            # The reason the code below works is that radius is negative.
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x + (radius / 2)
                self.center_y = self.center_y + (radius / 2)
                self.angle = -45
            else:
                self.center_x = self.center_x + radius
                self.center_y = self.center_y + radius
                self.angle = -90
            '''

        def _robotPointingZeroRadiusNegativeAngleNegative():
            self.center_x = self.center_x - (-1 * radius * math.sin(math.radians(abs(angle))))
            self.center_y = self.center_y + (-1 * radius * (1 - math.cos(math.radians(abs(angle)))))
            self.angle = -1 * angle

            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x + (radius/2)
                self.center_y = self.center_y - (radius/2)
                self.angle = 45
            else:
                self.center_x = self.center_x + radius
                self.center_y = self.center_y - radius
                self.angle = 90
            '''

        def _robotPointingNintyRadiusPositiveAnglePositive():
            self.center_x = self.center_x - (radius*(1 - math.cos(math.radians(angle))))
            self.center_y = self.center_y - (radius * math.sin(math.radians(angle)))
            self.angle = self.angle + angle
            if (self.angle > 180):
                self.angle = self.angle - 360

            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x - (radius/2)
                self.center_y = self.center_y - (radius/2)
                self.angle = 135
            else:    
                self.center_x = self.center_x - radius
                self.center_y = self.center_y - radius
                self.angle = 180
            '''

        def _robotPointingNintyRadiusPositiveAngleNegative():
            self.center_x = self.center_x + (radius*(1 - math.cos(math.radians(-1 * angle))))
            self.center_y = self.center_y - (radius * math.sin(math.radians(-1 * angle)))
            self.angle = self.angle + angle
            if (self.angle < -180):
                self.angle = 360 + self.angle
            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x + (radius/2)
                self.center_y = self.center_y - (radius/2)
                self.angle = 45
            else:
                self.center_x = self.center_x + radius
                self.center_y = self.center_y - radius
                self.angle = 0
            '''

        def _robotPointingNintyRadiusNegativeAnglePositive():
            self.center_x = self.center_x - (-1 * radius * (1 - math.cos(math.radians(angle))))
            self.center_y = self.center_y + (-1 * radius * math.sin(math.radians(angle)))
            self.angle = self.angle - angle
            if (self.angle < -180):
                self.angle = 360 + self.angle

            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x + (radius/2)
                self.center_y = self.center_y - (radius/2)
                self.angle = 45
            else:
                self.center_x = self.center_x + radius
                self.center_y = self.center_y - radius
                self.angle = 0
            '''
    
        def _robotPointingNintyRadiusNegativeAngleNegative():
            self.center_x = self.center_x + (-1 * radius*(1 - math.cos(math.radians(-1 * angle))))
            self.center_y = self.center_y + (-1 * radius * math.sin(math.radians(-1 * angle)))
            self.angle = self.angle - angle
            if (self.angle > 180):
                self.angle = self.angle - 360

            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x - (radius/2)
                self.center_y = self.center_y - (radius/2)
                self.angle = 135
            else:
                self.center_x = self.center_x - radius
                self.center_y = self.center_y - radius
                self.angle = 180
            '''

        def _robotPointing180RadiusPositiveAnglePositive():
            self.center_x = self.center_x - (radius * math.sin(math.radians(angle)))
            self.center_y = self.center_y + (radius*(1 - math.cos(math.radians(angle))))
            self.angle = self.angle + angle
            if (self.angle > 180):
                self.angle = self.angle - 360
            
            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x - (radius/2)
                self.center_y = self.center_y + (radius/2)
                self.angle = -135
            else:
                self.center_x = self.center_x - radius
                self.center_y = self.center_y + radius
                self.angle = -90
            '''

        def _robotPointing180RadiusPositiveAngleNegative():
            self.center_x = self.center_x - (radius * math.sin(math.radians(-1 * angle)))
            self.center_y = self.center_y - (radius*(1 - math.cos(math.radians(-1 * angle))))
            self.angle =  angle - self.angle
            if (self.angle < -180):
                self.angle = 360 + self.angle
            
            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x - (radius/2)
                self.center_y = self.center_y - (radius/2)
                self.angle = 135
            else:
                self.center_x = self.center_x - radius
                self.center_y = self.center_y - radius
                self.angle = 90
            '''

        def _robotPointing180RadiusNegativeAnglePositive():
            self.center_x = self.center_x + (-1 * radius * math.sin(math.radians(angle)))
            self.center_y = self.center_y + (-1 * radius*(1 - math.cos(math.radians(angle))))
            self.angle =  (-1 * self.angle) - angle
            if (self.angle > 180):
                self.angle = self.angle - 360
            
            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x - (radius/2)
                self.center_y = self.center_y - (radius/2)
                self.angle = 135
            else:
                self.center_x = self.center_x - radius
                self.center_y = self.center_y - radius
                self.angle = 90
            '''

        def _robotPointing180RadiusNegativeAngleNegative():
            self.center_x = self.center_x + (-1 * radius * math.sin(math.radians(-1 * angle)))
            self.center_y = self.center_y - (-1 * radius*(1 - math.cos(math.radians(-1 * angle))))
            self.angle =  self.angle - angle
            if (self.angle > 180):
                self.angle = self.angle - 360
            
            '''
            if (angle == 45 or angle == -45):
                self.center_x = self.center_x - (radius/2)
                self.center_y = self.center_y + (radius/2)
                self.angle = -135
            else:
                self.center_x = self.center_x - radius
                self.center_y = self.center_y + radius
                self.angle = -90
            '''

        # This code is first based on the current angle. And then we can only do 90 or 45 degree turns.
        if (current_angle == -90 or current_angle == 270):
            if (radius > 0):
                if (angle > 0):
                    _robotPointing90NegativeRadiusPositiveAnglePositive()
                else:
                    _robotPointing90NegativeRadiusPositiveAngleNegative()
            else:
                if (angle > 0):
                    _robotPointing90NegativeRadiusNegativeAnglePositive()
                else:
                    _robotPointing90NegativeRadiusNegativeAngleNegative()
                    
        elif (current_angle == 0 or current_angle == 360):
            if (radius > 0):
                if (angle > 0):
                    _robotPointingZeroRadiusPositiveAnglePositive()
                else:
                    _robotPointingZeroRadiusPositiveAngleNegative()
            else:
                if (angle > 0):
                    _robotPointingZeroRadiusNegativeAnglePositive()
                else:
                    _robotPointingZeroRadiusNegativeAngleNegative()
                    
        elif (current_angle == 90 or current_angle == 450):
            if (radius > 0):
                if (angle > 0):
                    _robotPointingNintyRadiusPositiveAnglePositive()
                else:
                    _robotPointingNintyRadiusPositiveAngleNegative()
            else:
                if (angle > 0):
                    _robotPointingNintyRadiusNegativeAnglePositive()
                else:
                    _robotPointingNintyRadiusNegativeAngleNegative()

        elif (current_angle == 180 or current_angle == -180):
            # set the angle to -180 in all cases
            self.angle = -180
            
            if (radius > 0):
                if (angle > 0):
                    _robotPointing180RadiusPositiveAnglePositive()
                else:
                    _robotPointing180RadiusPositiveAngleNegative()
            else:
                if (angle > 0):
                    _robotPointing180RadiusNegativeAnglePositive()
                else:
                    _robotPointing180RadiusNegativeAngleNegative()

        self.draw()

    def curve(self, radius=25, angle=90, speed=600, acceleration=200, 
              optional_simulator_end_point = None, optional_simulator_end_angle= None):
        if (angle == 180 or angle == -180):
            self._curve_internal(radius, int(angle / 2), speed, acceleration, optional_simulator_end_point, optional_simulator_end_angle)
            self._curve_internal(radius, int(angle / 2), speed, acceleration, optional_simulator_end_point, optional_simulator_end_angle)
        else:
            self._curve_internal(radius, angle, speed, acceleration, optional_simulator_end_point, optional_simulator_end_angle)

    def positionFrontBucket(self, position):
        self.front_bucket_position = position

    def positionBackBucket(self, position):
        self.back_bucket_position = position
        if (position == BACK_BUCKET_DOWN):
            self.back_bucket_down = True
        elif (position == BACK_BUCKET_UP):
            self.back_bucket_down = False

        elif (position == DROP_BALLS):
            self.back_bucket_down = False

    def setBackBucketDown(self):
        self.back_bucket_down = True

    def setBackBucketUp(self):
        self.back_bucket_down = False
        
    def draw(self):
        # If we dont want to draw the robot, just skip the drawing.
        if (DRAW_ROBOT == False):
            return

        turtle.clear()
        turtle.penup()
        turtle.goto(convertXCMToPixels(self.center_x),
                    convertYCMToPixels(self.center_y))
        turtle.setheading(self.convertTo360(self.angle))

        # Draw the color sensor.
        turtle.forward(convertYCMToPixels(2))
        turtle.pendown()
        turtle.circle(4)
        turtle.penup()
        turtle.forward(self.distance_to_front - convertYCMToPixels(2))
        
        turtle.pendown()
        turtle.write("Front: " + str(self.front_bucket_position), font=("Verdana", 20, "normal"))
        turtle.pencolor("red")
        
        # Draw the first half side
        turtle.right(90)

        # Draw circles to represent the two separators
        turtle.forward(convertXCMToPixels(4.5))
        turtle.circle(4)
        turtle.forward(convertXCMToPixels(7.5))
        turtle.circle(4)

        turtle.pencolor("blue")
        turtle.right(90)
        if (self.back_bucket_down == True):
            turtle.forward(self.distance_to_front + self.distance_to_back_with_bucket_down)
        else:
            turtle.forward(self.distance_to_front + self.distance_to_back)

        turtle.right(90)

        # Draw the circle for back separators
        turtle.forward(convertXCMToPixels(1.25))
        turtle.circle(4)
        turtle.forward(convertXCMToPixels(6.3))
        turtle.circle(4)
        turtle.forward(convertXCMToPixels(3.2))
        turtle.write("Back: " + str(self.back_bucket_position), font=("Verdana", 20, "normal"))
        turtle.forward(convertXCMToPixels(3.2))
        turtle.circle(4)
        turtle.forward(convertXCMToPixels(6.3))
        turtle.circle(4)
        turtle.forward(convertXCMToPixels(1.25))
        
        turtle.right(90)
        if (self.back_bucket_down == True):
            turtle.forward(self.distance_to_front + self.distance_to_back_with_bucket_down)
        else:
            turtle.forward(self.distance_to_front + self.distance_to_back)

        turtle.pencolor("red")
        turtle.right(90)
        
        # Draw circles to show the separators.
        turtle.circle(4)
        turtle.forward(convertXCMToPixels(7.5))
        turtle.circle(4)
        turtle.forward(convertXCMToPixels(2))

        # Now readjust the turtle to poitn in the direction of
        # the front of the robot.
        turtle.left(90)
        

def runSimulator():
    screen.mainloop()

def wait(time):
    return

def initializeGraphics(frontStart = True):
    global robot, SCALE, screen_width, screen_height
    game_mat = GAME_MAT
    SCALE = 0.95  # Adjust this value to fit your screen
    screen_width = int(MAT_WIDTH_PIXELS * SCALE)
    screen_height = int(MAT_HEIGHT_PIXELS * SCALE)

    # Resize the background image every time to match SCALE
    resized_bg = "resized_bg.png"
    img = Image.open(game_mat)
    img = img.resize((screen_width, screen_height), Image.Resampling.LANCZOS)
    img.save(resized_bg)
    game_mat_to_use = resized_bg

    screen.update()
    screen.title("WRO Mat")
    screen.screensize(canvwidth=screen_width, canvheight=screen_height, bg=None)
    screen.setup(width=(screen_width + 100), height=(screen_height + 100), startx=0, starty=0)
    screen.setworldcoordinates(0, 0, screen_width, screen_height)
    screen.bgpic(game_mat_to_use)
        
    screen.mode('world')
    turtle.pen(fillcolor="black", pencolor="blue", pensize=5)
    turtle.Mode = "world"
    turtle.speed(0)
    canvas = screen.getcanvas()
    canvas.itemconfig(screen._bgpic, anchor="sw")

    screen.delay(REFRESH_DELAY)
    screen.update()
    if frontStart:
        robot = Robot(initial_angle = -90, back_bucket_position=BACK_BUCKET_UP, front_bucket_position=FRONT_BUCKET_DOWN,
                      center_x = 20.5, center_y = 21.5)
    else:
        robot = Robot(initial_angle = 90, back_bucket_position=BACK_BUCKET_UP, front_bucket_position=FRONT_BUCKET_DOWN, 
                      center_x = 23.5, center_y = 19)
    
    screen.cv.config(cursor="crosshair")

def convertXCMToPixels(x):
    # Convert cm to pixels using original mat width
    return (MAT_WIDTH_PIXELS * x) / MAT_WIDTH_CM * SCALE

def convertYCMToPixels(y):
    # Convert cm to pixels using original mat height
    return (MAT_HEIGHT_PIXELS * y) / MAT_HEIGHT_CM * SCALE

def convertPixelXtoCM(x):
    # Convert pixels to cm using original mat width
    return (MAT_WIDTH_CM * x) / (MAT_WIDTH_PIXELS * SCALE)

def convertPixelYtoCM(y):
    # Convert pixels to cm using original mat height
    return (MAT_HEIGHT_CM * y) / (MAT_HEIGHT_PIXELS * SCALE)

def initializeAndWaitForRobotReady(waitForButtonPush=True):
    initializeGraphics(frontStart=True)
    initializedistancemeasuringtool()
    
def resetHeading(heading):
    if robot is not None:
        robot.setAngle(heading)
        robot.draw()

def waitForLeftButtonPress():
    return

def positionFrontBucket(position, speed=500, acceleration=1000, wait=True, stop = Stop.BRAKE,stallDetect=False):
    if robot is not None:
        robot.positionFrontBucket(position)
    return

def positionBackBucket(position, speed=200, acceleration=400, wait=True,stallDetect=False):
    if robot is not None:
        robot.positionBackBucket(position)
    return

def curve(radius, angle, speed = 300, acceleration = 450, deceleration = 0, 
          optional_simulator_end_point = None, 
          optional_simulator_end_angle= None):
    if robot is not None:
        robot.curve(radius, angle, speed = 300, 
                    optional_simulator_end_point = optional_simulator_end_point, 
                    optional_simulator_end_angle = optional_simulator_end_angle)

def curve_with_load_detect_to_get_out_park_b(radius, angle, speed = 300, acceleration = 450, deceleration = 0, 
          optional_simulator_end_point = None, 
          optional_simulator_end_angle= None):
    if robot is not None:
        robot.curve(radius, angle, speed = 300, 
                    optional_simulator_end_point = optional_simulator_end_point, 
                    optional_simulator_end_angle = optional_simulator_end_angle)

def curve_with_accel_options(radius, angle, speed=300, acceleration=450, deceleration=0,
                             dont_accelerate=False, dont_decelerate=False,
                             optional_simulator_end_point=None, optional_simulator_end_angle=None):
    if robot is not None:
        robot.curve(radius, angle, speed = 300, 
                    optional_simulator_end_point = optional_simulator_end_point, 
                    optional_simulator_end_angle = optional_simulator_end_angle)

def turnToAngle(targetAngle, speed=400, turn_acceleration=400, turn_deceleration=450, 
                right_correction=-0.1, left_correction = -0.1, forceTurn = FORCETURN_NONE, 
                oneWheelTurn = False,
                then=Stop.BRAKE):
    if robot is not None:
        robot.turnToAngle(targetAngle, speed=400)

def followBlackLinePID(distanceInMM, speed, edge, controlColor = 63, 
                       kp=0.5, ki=0, kd=0, correctionBasedSpeed = False, slowStart = True, 
                       slowDown = True, printDebugMessages=False,
                       additionalFunctionToCall = None,
                       slowSpeedOverride = 100):
    if robot is not None:
        robot.followBlackLinePID(distanceInMM, speed, edge, controlColor)

def gyroStraightWithDrive(distance, speed, backward = False, targetAngle = 0, 
                          multiplier=4,
                          slowDown=True, printDebugMesssages=False,
                          tillBlackLine = False,
                          detectStall = False,
                          stop = Stop.BRAKE,
                          alternativeFunctionToCall = None,
                          slowSpeedOverride = 50,
                          optional_simulator_max_distance_from_wall = None, flushForSimulator = False):
    if robot is not None:
        robot.gyroStraightWithDrive(distance, targetAngle, speed, backward, 
            tillBlackLine, optional_simulator_max_distance_from_wall, flushForSimulator)

def flushUsingStallDetect(distance, max_load = 100, speed=200, backward=False, acceleration=500, min_dist = None):
    if (distance < 0):
        backward = True
        distance = distance * -1
    if robot is not None:
        robot.flushUsingStallDetect(distance, max_load, speed, backward, acceleration)

def resetPosition(x, y, angle):
    if robot is not None:
        robot.resetPosition(x, y, angle)

def draw_line_on_click(x, y):
    global dturtle, button, dataButton
    global startPosX
    global startPosY
    print("Action: " + str(MeasureAction) +" (x_cm:" + str(convertPixelXtoCM(x)) + ", y_cm:" + str(convertPixelYtoCM(y)) + ")")
    
    if(startPosX == -100 and startPosY == -100):
        startPosX = x
        startPosY = y
        button.configure(text="Choose end position")
        dataButton.configure(text="startX= " + str(startPosX)+ " startY= " + str(startPosY))
    else:
        if(MeasureAction == HORIZONTAL_LINE):
            y=startPosY
        if(MeasureAction == VERTICAL_LINE):
            x = startPosX
        dturtle.penup()
        dturtle.setpos(startPosX, startPosY)
        dturtle.pendown()
        dturtle.goto(x,y)
        #dturtle.hideturtle()
        endX = convertPixelXtoCM(x)
        endY = convertPixelYtoCM(y)
        startX = convertPixelXtoCM(startPosX)
        startY = convertPixelYtoCM(startPosY)
        distance = math.sqrt((endX-startX) * (endX-startX) + (endY-startY) * (endY-startY))
        if(endX!=startX):
            slope = (endY-startY)/(endX-startX)
            if(slope!=0):
                angle = math.degrees(math.atan(slope))
                if(endX>startX and endY > startY): angle = angle * -1
                if(endX>startX and endY < startY): angle = angle * -1
                if(endX<startX and endY > startY): angle = -(180 + angle)
                if(endX<startX and endY < startY): angle = 180 - angle
            elif(endX>startX):
                angle = 0
            else:
                angle = 180


        elif(endY>startY):
            angle = -90
        else:
            angle = 90

        dataButton.configure(text="Distance= " + str(math.ceil(distance)) + " angle= " + str(math.ceil(angle)))
        button.configure(text="Clear Line")
        startPosX = -100
        startPosY = -100
    #dataButton.configure(text="X= " + str(convertPixelXtoCM(x))+ " startY= " + str(convertPixelYtoCM(y)))
          
def OnClickMeasureButton():
    global dturtle, button, dataButton

    dturtle.clear()    
    button.configure(text="Click on First point")

def on_select(event):
    global MeasureAction
    selected_value=event
    if(selected_value == "Distance"):
        MeasureAction = DISTANCE
    if(selected_value == "Horizontal Line"):
        MeasureAction = HORIZONTAL_LINE
    if(selected_value == "Vertical Line"):
        MeasureAction = VERTICAL_LINE

def initializedistancemeasuringtool():
    global dturtle, button, dataButton

    canvas = screen.getcanvas()
    #Button is used for clearing line and databutton for showing the X Y coordinates and distance of the line.
    button = Button(canvas.master, text="Measure Distance", command=OnClickMeasureButton)
    dataButton = Button(canvas.master,text = "")
    screen.onclick(draw_line_on_click)
    
    dturtle.Mode = "standard"
    dturtle.speed(0)
    dturtle.pen(fillcolor="black", pencolor="magenta", pensize=8)
    dataButton.pack()
    dataButton.place(x=180,y=5)

    button.pack()
    button.place(x=40, y=5)  # place the button anywhere on the screen

    OptionList = ["Distance","Horizontal Line", "Vertical Line"]
    variable = tk.StringVar(canvas.master)
    opt = tk.OptionMenu(canvas.master,variable,*OptionList,command=on_select)
    # This is according to world co-ordinates so 0,0 is the bottom left of the screen.
    canvas.create_window(MAT_WIDTH_PIXELS / 2, -MAT_HEIGHT_PIXELS-10,window=opt)
    opt.config(width=20,font=('Helvetica',12))

def runWithTiming(function,name):
    return function()

def simulatorOverrideBlockString(blockString):
    return "gbggbg"

def getAverageLoad():
    return 50