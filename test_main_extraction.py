# -*- coding: utf-8 -*-
"""
Test script to demonstrate waypoint extraction from main.py
This shows how to use the new waypoint extraction features.
"""

import tkinter as tk
from path_drawer_with_edit import PathDrawer

def test_main_extraction():
    """Test extracting waypoints from main.py"""
    print("Testing waypoint extraction from main.py...")
    
    # Create the path drawer application
    root = tk.Tk()
    app = PathDrawer(root)
    
    # Simulate loading main.py content
    main_content = """
# Example robot program similar to main.py
hub.imu.reset_heading(180)

def goFromHomeToBallPit():
    gyroStraightWithDrive(distance=13, speed=900, targetAngle=180, backward=True, slowDown=False)
    curve(radius=-165, angle=180, speed=400)
    gyroStraightWithDrive(distance=3, speed=200, backward=True, targetAngle=180)

def getBallsFromBallPit():
    setPickersPosition(PICK_GREEN, speed=300, wait=False)
    gyroStraightWithDrive(distance=8, speed=200, targetAngle=0)
    setBallPickerPosition(PICK_BALLS, speed=300, wait=True)
    wait(50)
    drive_base.straight(-130)
    drive_base.straight(50)
    setBallPickerPosition(SLIDE_BALL, speed=100, wait=False)
    gyroStraightWithDrive(distance=50, speed=400)
    setBallPickerPosition(PICK_BALLS, speed=150, wait=False)
    turnToAngle(targetAngle=0, speed=100)
    gyroStraightWithDrive(distance=12, speed=100, backward=True, targetAngle=0)
    wait(100)
    setPickersPosition(ALL_UP, wait=False)
    gyroStraightWithDrive(distance=5, speed=250, targetAngle=0)
    wait(100)

def dropBalls():
    curve_with_accel_options(radius=110, angle=-90, speed=400, dont_decelerate=True)
    drive_base.straight(205)
    curve_with_accel_options(radius=110, angle=-90, speed=800, dont_accelerate=True, dont_decelerate=True)
    drive_base.straight(52)
    setPickersPosition(HALF_LIFT_LID, speed=500, wait=True)
    gyroStraightWithDrive(distance=4, speed=600, targetAngle=180, slowDown=False)
    setPickersPosition(LIFT_LID, speed=1000, wait=False)
    gyroStraightWithDrive(distance=6, speed=600, targetAngle=180, slowDown=False)
    setBallPickerPosition(DROP_BALLS, speed=200, wait=True)

# Main execution
goFromHomeToBallPit()
getBallsFromBallPit()
dropBalls()
"""
    
    # Test the code analysis extraction
    print("Testing code analysis extraction...")
    waypoints = app.parse_and_simulate_robot_code(main_content, app.planner)
    
    print(f"Extracted {len(waypoints)} waypoints:")
    for i, wp in enumerate(waypoints):
        print(f"  Waypoint {i}: ({wp['x']:.1f}, {wp['y']:.1f}) @ {wp['heading']:.1f}° - {wp['command']['type']}")
    
    # Show the waypoint editor
    print("Opening waypoint editor...")
    app.show_waypoint_editor_with_waypoints(waypoints)
    
    # Start the GUI
    root.mainloop()

if __name__ == "__main__":
    test_main_extraction() 