# Interactive Waypoint Editor for Path Planner

This document explains how to use the new interactive waypoint editor feature that allows you to edit robot paths visually and generate updated code.

## Overview

The interactive waypoint editor provides a graphical user interface (GUI) for:
- Visualizing robot paths
- Adding, editing, and deleting waypoints
- Generating updated Python code
- Saving and loading paths

## How to Use

### Method 1: From Main.py (Recommended)

1. Open `main.py`
2. At the bottom of the file, you'll see these options:
   ```python
   # Uncomment one of the following lines to run:
   
   # Run the main robot program with timing
   #runWithTiming(main, "BOBAAAA🔥🔥🧋🧋:")
   
   # Run the main robot program without timing
   #main()
   
   # Launch the interactive path editor (NEW FEATURE!)
   # This allows you to edit waypoints and generate new code
   interactive_path_editor()
   ```

3. Make sure `interactive_path_editor()` is uncommented and the others are commented out
4. Run `main.py`:
   ```bash
   python main.py
   ```

This will:
- Create a path based on your main function
- Launch the interactive waypoint editor
- Allow you to edit the waypoints and generate new code

### Method 2: Using the Example Script

1. Run the example script:
   ```bash
   python example_path_editor.py
   ```

2. Choose from three example paths:
   - Simple square path
   - Complex path with curves and line following
   - Empty path (add waypoints manually)

### Method 3: Direct Usage

```python
from path_planner import PathPlanner

# Create a path planner
planner = PathPlanner()

# Add some waypoints
planner.reset_position(0, 0, 0)
planner.drive_straight(50, speed=400)
planner.turn_to_angle(90, speed=400)
planner.drive_straight(50, speed=400)

# Launch the interactive editor
planner.interactive_waypoint_editor()
```

## Features of the Waypoint Editor

### Visual Interface

The editor shows:
- **Field boundaries**: The robot competition field
- **Grid lines**: For easy measurement
- **Robot path**: Blue line showing the current path
- **Waypoints**: Green dots with robot representations
- **Selected waypoint**: Red dot (when a waypoint is selected)
- **Direction arrows**: Showing robot heading at each waypoint
- **Status bar**: Shows mouse position and current action
- **Instructions**: Displayed on the plot for quick reference

### Waypoint Management

#### Adding Waypoints
**Method 1: Drag to Add**
1. Click and drag in empty space on the plot
2. Drag in the direction you want the robot to face
3. Release to create a new waypoint

**Method 2: Right-Click Menu**
1. Right-click in empty space on the plot
2. Select "Add Waypoint Here" or "Add Waypoint with Dialog"

**Method 3: Button**
1. Click "Add Waypoint" button
2. Enter X, Y coordinates (in cm) and heading (in degrees)
3. Click "Add"

#### Editing Waypoints
**Method 1: Drag to Edit (Recommended)**
1. Click and drag any waypoint on the plot
2. The waypoint will move in real-time
3. Release to set the new position

**Method 2: Right-Click Menu**
1. Right-click on a waypoint
2. Select "Edit Waypoint X"

**Method 3: List Selection**
1. Select a waypoint from the list (it will turn red on the plot)
2. Click "Edit Waypoint" button
3. Modify the coordinates and heading
4. Click "Update"

#### Deleting Waypoints
**Method 1: Right-Click Menu**
1. Right-click on a waypoint
2. Select "Delete Waypoint X"

**Method 2: List Selection**
1. Select a waypoint from the list
2. Click "Delete Waypoint" button
3. Confirm the deletion

#### Inserting Waypoints
1. Right-click on a waypoint
2. Select "Insert Waypoint Before" or "Insert Waypoint After"
3. A new waypoint will be inserted at the cursor position

### Code Generation

1. Click "Generate Code" button
2. A dialog will appear showing the generated Python code
3. You can:
   - Copy the code to clipboard
   - Save the code to a file
   - Close the dialog

The generated code includes:
- All necessary imports
- Function definition with your path
- Proper robot commands (drive_straight, turn_to_angle, curve, etc.)
- Comments showing command types

### Save/Load Paths

#### Saving Paths
1. Click "Save Path" button
2. Choose a location and filename
3. The path will be saved as a JSON file

#### Loading Paths
1. Click "Load Path" button
2. Select a previously saved JSON file
3. The path will be loaded and displayed

## Understanding the Interface

### Waypoint List
The left panel shows all waypoints with:
- Waypoint number
- X, Y coordinates
- Heading angle
- Associated command type (if any)

### Plot Area
The right panel shows:
- **Field**: 236cm × 114cm competition field
- **Robot**: Red/green rectangles showing robot position and orientation
- **Path**: Blue line connecting waypoints
- **Statistics**: Total distance and number of waypoints

### Coordinate System
- **Origin**: Bottom-left corner of the field
- **X-axis**: Horizontal (0 to 236 cm)
- **Y-axis**: Vertical (0 to 114 cm)
- **Heading**: 0° = right, 90° = down, 180° = left, 270° = up

## Tips for Effective Use

1. **Use drag-to-edit**: Click and drag waypoints directly on the plot for quick editing
2. **Drag to add waypoints**: Click and drag in empty space to create new waypoints
3. **Right-click for options**: Right-click on waypoints or empty space for context menus
4. **Watch the status bar**: It shows mouse position and current actions
5. **Use the grid**: The grid lines help with precise positioning
6. **Check robot orientation**: The robot rectangles show the heading at each waypoint
7. **Test incrementally**: Make small changes and generate code to test
8. **Save frequently**: Save your work as you make progress

## Generated Code Format

The generated code follows this format:
```python
def edited_path():
    """
    Generated path function.
    Total distance: X.X cm
    Estimated time: X.X seconds
    """
    # Import required constants
    from utils import Stop, ALL_UP, PICK_BALLS, ...
    
    # Command 1: drive_straight
    gyroStraightWithDrive(distance=50, speed=400)
    
    # Command 2: turn_to_angle
    turnToAngle(targetAngle=90, speed=400)
    
    # ... more commands
```

## Troubleshooting

### Common Issues

1. **GUI doesn't appear**: Make sure you have tkinter installed
2. **Plot doesn't update**: Try clicking on the plot area to refresh
3. **Code generation fails**: Check that all waypoints have valid coordinates
4. **File save/load errors**: Ensure you have write permissions in the directory

### Dependencies

Make sure you have these Python packages installed:
- `matplotlib`
- `numpy`
- `tkinter` (usually included with Python)

Install missing packages with:
```bash
pip install matplotlib numpy
```

## Advanced Features

### Custom Field Images
You can add custom field images by modifying the `visualize_path` method in `path_planner.py`.

### Additional Command Types
The system supports all robot commands:
- `drive_straight`
- `turn_to_angle`
- `curve` / `awaitarc`
- `follow_line`
- `set_pickers_position`
- `set_ball_picker_position`
- `wait`

### Extending the Editor
You can extend the editor by adding new features to the `WaypointEditor` class in `path_planner.py`. 