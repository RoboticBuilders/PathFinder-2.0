# -*- coding: utf-8 -*-
"""
Test program for waypoint extraction feature.
This program demonstrates various path planning commands.
"""

def test_path():
    """A simple test path that can be extracted for waypoint editing."""
    # This would normally use the actual robot functions
    # For testing, we'll create a simple path
    
    # Start at origin
    print("Starting at (0, 0, 0)")
    
    # Drive forward 50cm
    print("Driving forward 50cm")
    # gyroStraightWithDrive(distance=50, speed=400)
    
    # Turn right 90 degrees
    print("Turning right 90 degrees")
    # turnToAngle(targetAngle=90, speed=400)
    
    # Drive forward 30cm
    print("Driving forward 30cm")
    # gyroStraightWithDrive(distance=30, speed=400)
    
    # Turn left 45 degrees
    print("Turning left 45 degrees")
    # turnToAngle(targetAngle=-45, speed=400)
    
    # Drive forward 40cm
    print("Driving forward 40cm")
    # gyroStraightWithDrive(distance=40, speed=400)
    
    print("Path completed!")

def main():
    """Main function to run the test path."""
    test_path()

if __name__ == "__main__":
    main() 