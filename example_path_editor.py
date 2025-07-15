#!/usr/bin/env python3
"""
Example script demonstrating the interactive waypoint editor feature.
This script shows how to create a path, edit waypoints, and generate code.
"""

from path_planner import PathPlanner

def create_simple_example_path():
    """
    Create a simple example path to demonstrate the waypoint editor.
    """
    planner = PathPlanner()
    
    # Start at origin
    planner.reset_position(0, 0, 0)
    
    # Create a simple square path
    planner.drive_straight(50, speed=400)  # Drive forward
    planner.turn_to_angle(90, speed=400)   # Turn right
    planner.drive_straight(50, speed=400)  # Drive forward
    planner.turn_to_angle(180, speed=400)  # Turn right
    planner.drive_straight(50, speed=400)  # Drive forward
    planner.turn_to_angle(270, speed=400)  # Turn right
    planner.drive_straight(50, speed=400)  # Drive forward
    planner.turn_to_angle(0, speed=400)    # Turn back to start
    
    return planner

def create_complex_example_path():
    """
    Create a more complex example path with curves and different commands.
    """
    planner = PathPlanner()
    
    # Start at origin
    planner.reset_position(0, 0, 0)
    
    # Drive forward
    planner.drive_straight(30, speed=400)
    
    # Turn right with curve
    planner.curve(100, 90, speed=300)
    
    # Follow a line
    planner.follow_line(100, speed=200, edge='left')
    
    # Set picker position
    planner.set_pickers_position('ALL_UP')
    
    # Turn back
    planner.turn_to_angle(180, speed=400)
    
    # Drive back
    planner.drive_straight(80, speed=400, backward=True)
    
    return planner

def main():
    """
    Main function to demonstrate the interactive waypoint editor.
    """
    print("Path Planner - Interactive Waypoint Editor Demo")
    print("=" * 50)
    print()
    print("Choose an example path to edit:")
    print("1. Simple square path")
    print("2. Complex path with curves and line following")
    print("3. Create empty path and add waypoints manually")
    print()
    
    choice = input("Enter your choice (1-3): ").strip()
    
    if choice == "1":
        print("Creating simple square path...")
        planner = create_simple_example_path()
    elif choice == "2":
        print("Creating complex path...")
        planner = create_complex_example_path()
    elif choice == "3":
        print("Creating empty path...")
        planner = PathPlanner()
        planner.reset_position(0, 0, 0)
    else:
        print("Invalid choice. Using simple square path.")
        planner = create_simple_example_path()
    
    print()
    print("Launching interactive waypoint editor...")
    print("In the editor, you can:")
    print("- View the current path")
    print("- Add new waypoints")
    print("- Edit existing waypoints")
    print("- Delete waypoints")
    print("- Generate updated code")
    print("- Save/load paths")
    print()
    print("Press Enter to continue...")
    input()
    
    # Launch the interactive editor
    planner.interactive_waypoint_editor()

if __name__ == "__main__":
    main() 