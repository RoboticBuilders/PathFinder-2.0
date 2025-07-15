# Path Drawer with Waypoint Editing Features

This enhanced version of the Path Drawer includes comprehensive waypoint editing capabilities that allow you to run programs, extract waypoints from code, edit them visually, and generate new code based on the edited waypoints.

## New Features

### 1. Waypoint Editing Section
The interface now includes a dedicated "Waypoint Editing" section with the following buttons:

- **Extract Waypoints from Code**: Extracts waypoints from the current path and opens the waypoint editor
- **Edit Waypoints**: Opens the waypoint editor for the current path
- **Generate Code from Waypoints**: Generates code from edited waypoints
- **Run Program & Extract Waypoints**: Runs a Python program and extracts waypoints from its execution

### 2. Program Execution and Waypoint Extraction

#### Running Programs
1. Click "Run Program & Extract Waypoints"
2. Select a Python file to run (like `main.py`)
3. The program editor dialog will open showing the file content
4. Click "Run Program" to execute the program
5. The system will look for functions ending with `_path` or named `main`
6. Waypoints will be extracted from the program execution

#### Code Analysis (NEW - Enhanced for Real Robot Code)
1. In the program editor dialog, click "Extract Waypoints from Code"
2. The system analyzes the code for real robot commands like:
   - `gyroStraightWithDrive(distance=X, speed=Y, targetAngle=Z, backward=True/False)`
   - `turnToAngle(targetAngle=X, speed=Y, oneWheelTurn=True/False)`
   - `curve(radius=X, angle=Y, speed=Z)`
   - `curve_with_accel_options(radius=X, angle=Y, speed=Z, dont_accelerate=True/False)`
   - `drive_base.straight(X)` and `drive_base.turn(X)`
   - `setPickersPosition(POSITION, speed=X, wait=True/False)`
   - `setBallPickerPosition(POSITION, speed=X, wait=True/False)`
   - `wait(X)` for timing delays
   - `hub.imu.reset_heading(X)` for heading initialization
3. Waypoints are calculated by simulating the robot movements through the code
4. Each waypoint includes the original command information and line number

### 3. Interactive Waypoint Editor

The waypoint editor provides a comprehensive interface for editing waypoints:

#### Features:
- **Visual Display**: Shows waypoints on a field map with robot visualization
- **Waypoint List**: Displays all waypoints with coordinates and headings
- **Add Waypoints**: Add new waypoints at specific coordinates
- **Edit Waypoints**: Modify existing waypoint positions and headings
- **Delete Waypoints**: Remove unwanted waypoints
- **Generate Code**: Create new robot code from the edited waypoints

#### Usage:
1. Waypoints are displayed as numbered markers on the field
2. Select a waypoint from the list to edit it
3. Use the "Add Waypoint" button to create new waypoints
4. Use "Edit Waypoint" to modify selected waypoints
5. Use "Delete Waypoint" to remove selected waypoints
6. Click "Generate Code" to create new robot code

### 4. Code Generation from Waypoints

After editing waypoints, you can generate new robot code:

1. The system creates a new PathPlanner instance with the edited waypoints
2. Code is generated using the standard code generation features
3. The generated code is displayed in a dialog with copy functionality

## Example Workflow

### Basic Workflow:
1. **Create a Path**: Use the path drawer to create an initial path
2. **Extract Waypoints**: Click "Extract Waypoints from Code"
3. **Edit Waypoints**: Modify waypoint positions and headings as needed
4. **Generate New Code**: Click "Generate Code from Waypoints"
5. **Use the Code**: Copy and use the generated code in your robot program

### Advanced Workflow with Real Robot Code:
1. **Load Robot Program**: Use "Run Program & Extract Waypoints" to load `main.py` or similar
2. **Extract Waypoints**: Click "Extract Waypoints from Code" to analyze the robot commands
3. **Review Waypoints**: See all robot movements visualized on the field map
4. **Edit Waypoints**: Modify positions, headings, or add new waypoints
5. **Generate Optimized Code**: Create new robot code based on the edits
6. **Test and Iterate**: Use the generated code and refine as needed

### Real-World Example:
1. **Load main.py**: Select your robot's main program file
2. **Analyze Commands**: The system parses all `gyroStraightWithDrive`, `turnToAngle`, `curve`, etc.
3. **Visualize Path**: See the complete robot path on the field
4. **Optimize Waypoints**: Adjust waypoints to improve efficiency or avoid obstacles
5. **Generate New Code**: Create optimized robot code with the same functionality

## File Structure

- `path_drawer_with_edit.py`: Main application with waypoint editing features
- `test_path_program.py`: Example program for testing waypoint extraction
- `test_main_extraction.py`: Test script demonstrating extraction from real robot code
- `main.py`: Example real robot program (your actual robot code)
- `path_planner.py`: Core path planning functionality

## Technical Details

### Waypoint Extraction Methods:
1. **Execution-based**: Runs the program and captures waypoints from PathPlanner instances
2. **Code analysis**: Parses the code to find path planning commands and calculates waypoints

### Waypoint Editor Features:
- Real-time visualization with matplotlib
- Interactive waypoint manipulation
- Coordinate and heading editing
- Path visualization with robot representation

### Code Generation:
- Supports all standard path planning commands
- Maintains command parameters and structure
- Generates clean, readable robot code

## Usage Tips

1. **Start Simple**: Begin with basic paths to understand the workflow
2. **Test Programs**: Use the provided test program to experiment with features
3. **Save Work**: Use the save/load features to preserve your work
4. **Iterate**: Edit waypoints multiple times to optimize your paths
5. **Validate**: Always test generated code before using it on the robot

## Troubleshooting

### Common Issues:
- **No waypoints extracted**: Ensure your program contains path planning functions
- **Program execution errors**: Check that your program doesn't have syntax errors
- **Waypoint editor not opening**: Make sure you have a path loaded first

### Error Messages:
- "No path functions found": Add functions ending with `_path` or named `main`
- "No waypoints to generate code from": Create a path first before generating code
- "Failed to load program": Check file permissions and Python syntax

## Future Enhancements

Potential improvements for future versions:
- Drag-and-drop waypoint editing
- Automatic path optimization
- Integration with robot simulation
- Support for more complex path patterns
- Real-time robot position tracking 