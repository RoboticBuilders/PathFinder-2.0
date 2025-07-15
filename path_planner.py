# -*- coding: utf-8 -*-
"""
Path Planner for LEGO SPIKE Prime Robot
A comprehensive tool for planning, visualizing, and generating robot movement paths.
"""

import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch
import numpy as np
from typing import List, Tuple, Dict, Optional, Union
import json
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

class PathPlanner:
    """
    A comprehensive path planning tool for LEGO SPIKE Prime robot movements.
    Supports visualization and code generation for all movement commands.
    """
    
    def __init__(self, robot_width=13.5, robot_length=13.5, grid_size=1.0):
        """
        Initialize the path planner.
        
        Args:
            robot_width: Width of robot in cm
            robot_length: Length of robot in cm  
            grid_size: Grid size for visualization in cm
        """
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.grid_size = grid_size
        
        # Robot state tracking
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0  # degrees, 0 = positive X axis
        
        # Path history
        self.path_history = []
        self.commands = []
        
        # Field dimensions (adjust as needed)
        self.field_width = 236  # cm
        self.field_height = 114  # cm
        
        # Color constants from utils.py
        self.BLACK_COLOR = 30
        self.WHITE_COLOR = 60
        
    def reset_position(self, x=0.0, y=0.0, heading=0.0):
        """Reset robot position and clear path history."""
        self.x = x
        self.y = y
        self.heading = heading
        self.path_history = [(x, y, heading)]
        self.commands = []
        
    def _add_to_history(self):
        """Add current position to path history."""
        self.path_history.append((self.x, self.y, self.heading))
        
    def _update_position_after_drive(self, distance, backward=False):
        """Update robot position after driving straight."""
        if backward:
            distance = -distance
            
        # Convert heading to radians
        heading_rad = math.radians(self.heading)
        
        # Calculate new position
        self.x += distance * math.cos(heading_rad)
        self.y += distance * math.sin(heading_rad)
        
        self._add_to_history()
        
    def _update_position_after_turn(self, angle_change):
        """Update robot position after turning."""
        self.heading += angle_change
        self._add_to_history()
        
    def _update_position_after_curve(self, radius, angle):
        """Update robot position after curve movement."""
        # Convert to radians
        angle_rad = -1 * math.radians(angle)
        
        if radius == 0:  # Straight line
            self._update_position_after_drive(angle * math.pi / 180 * abs(radius))
            return
            
        # Convert radius from mm to cm for position calculations
        radius_cm = radius / 10.0
        
        # Calculate center of curvature
        # The robot is on the circumference of a circle with given radius
        heading_rad = math.radians(self.heading)
        
        # WRO coordinate system: 0° = right (+X), 90° = down (+Y), 180° = left (-X), 270° = up (-Y)
        # Front/back is determined by sign of radius, left/right is determined by sign of angle
        # Positive radius = forward, negative radius = backward
        # Positive angle = right turn, negative angle = left turn
        
        # Calculate center of curvature
        # For forward movement (positive radius): center is perpendicular to robot's heading
        # For backward movement (negative radius): center is opposite side
        if radius > 0:  # Forward
            center_x = self.x - radius_cm * math.sin(heading_rad)
            center_y = self.y + radius_cm * math.cos(heading_rad)
        else:  # Backward
            center_x = self.x + radius_cm * math.sin(heading_rad)
            center_y = self.y - radius_cm * math.cos(heading_rad)
            
        # Calculate new position after traveling along the arc
        new_heading_rad = heading_rad + angle_rad
        
        # New position is on the circle at the new angle
        if radius > 0:  # Forward
            self.x = center_x + radius_cm * math.sin(new_heading_rad)
            self.y = center_y - radius_cm * math.cos(new_heading_rad)
        else:  # Backward
            self.x = center_x - radius_cm * math.sin(new_heading_rad)
            self.y = center_y + radius_cm * math.cos(new_heading_rad)
            
        self.heading = math.degrees(new_heading_rad)
        self._add_to_history()
        
    def drive_straight(self, distance, speed=400, backward=False, target_angle=None, 
                      till_black_line=False, till_white_line=False, detect_stall=False,
                      stop_when_load_above=0, slow_down=True, slow_speed_override=50):
        """
        Add a straight drive command to the path.
        
        Args:
            distance: Distance to drive in cm
            speed: Speed in mm/s
            backward: Whether to drive backward
            target_angle: Target heading angle (for gyro correction)
            till_black_line: Stop when black line detected
            till_white_line: Stop when white line detected
            detect_stall: Stop when stall detected
            stop_when_load_above: Stop when load exceeds this value
            slow_down: Whether to slow down at end
            slow_speed_override: Slow speed override
        """
        # Update position
        self._update_position_after_drive(distance, backward)
        
        if -target_angle == -360:
            target_angle = 0

        else:
            target_angle = -target_angle

        # Create command
        cmd = {
            'type': 'drive_straight',
            'distance': distance,
            'speed': speed,
            'backward': backward,
            'target_angle': target_angle if target_angle is not None else None,
            'till_black_line': till_black_line,
            'till_white_line': till_white_line,
            'detect_stall': detect_stall,
            'stop_when_load_above': stop_when_load_above,
            'slow_down': slow_down,
            'slow_speed_override': slow_speed_override
        }
        
        self.commands.append(cmd)
        
    def turn_to_angle(self, target_angle, speed=400, force_turn=None, one_wheel_turn=False):
        """
        Add a turn to angle command to the path.
        
        Args:
            target_angle: Target angle in degrees
            speed: Turn speed
            force_turn: Force turn direction ('left', 'right', or None)
            one_wheel_turn: Whether to use one wheel turn
        """
        # Calculate angle change
        current_angle = self.heading
        angle_diff = target_angle - current_angle
        
        # Find shortest path
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
            
        # Update position
        self._update_position_after_turn(angle_diff)
        
        # Create command
        cmd = {
            'type': 'turn_to_angle',
            'target_angle': -target_angle,
            'speed': speed,
            'force_turn': force_turn,
            'one_wheel_turn': one_wheel_turn
        }
        
        self.commands.append(cmd)
        
    def awaitarc(self, radius, angle=None, distance=None, speed=300, then="HOLD", wait=True):
        """
        Add an arc command to the path (similar to awaitarc).
        
        Args:
            radius: Radius of the circle in mm (positive = right turn, negative = left turn)
            angle: Angle to drive along the circle in degrees (optional)
            distance: Distance to drive along the circle in mm (optional)
            speed: Speed in mm/s
            then: What to do after coming to a standstill ("HOLD", "BRAKE", "COAST")
            wait: Wait for the maneuver to complete before continuing
            
        Raises:
            ValueError: If both angle and distance are specified, or if neither is specified, or if radius is zero
        """
        # Validate parameters
        if angle is not None and distance is not None:
            raise ValueError("You must specify angle or distance, but not both")
        if angle is None and distance is None:
            raise ValueError("You must specify either angle or distance")
        if radius == 0:
            raise ValueError("The radius cannot be zero. Use turn() for in-place turns")
        
        # Calculate the angle if distance is provided
        if distance is not None:
            # Convert distance to angle: angle = distance / radius (in radians, then to degrees)
            angle = math.degrees(distance / radius)
        
        # Update position using the existing curve logic
        self._update_position_after_curve(radius, angle)
        
        # Create command
        cmd = {
            'type': 'awaitarc',
            'radius': radius,
            'angle': angle,
            'distance': distance,
            'speed': speed,
            'then': then,
            'wait': wait
        }
        
        self.commands.append(cmd)
        
    def curve(self, radius, angle, speed=300, acceleration=450, deceleration=0, 
              dont_accelerate=False, dont_decelerate=False):
        """
        Add a curve command to the path (legacy method).
        
        Args:
            radius: Curve radius in mm (positive = right turn, negative = left turn)
            angle: Angle to turn in degrees
            speed: Speed in mm/s
            acceleration: Acceleration in mm/s²
            deceleration: Deceleration in mm/s²
            dont_accelerate: Skip acceleration phase
            dont_decelerate: Skip deceleration phase
        """
        # Update position
        self._update_position_after_curve(radius, angle)
        
        # Create command
        cmd = {
            'type': 'curve',
            'radius': radius,
            'angle': angle,
            'speed': speed,
            'acceleration': acceleration,
            'deceleration': deceleration,
            'dont_accelerate': dont_accelerate,
            'dont_decelerate': dont_decelerate
        }
        
        self.commands.append(cmd)
        
    def follow_line(self, distance, speed, edge='left', control_color=63, 
                   kp=0.5, ki=0, kd=0, slow_start=True, slow_down=True,
                   slow_speed_override=100, start_cm_slow=0):
        """
        Add a line following command to the path.
        
        Args:
            distance: Distance to follow line in mm
            speed: Speed in mm/s
            edge: Which edge to follow ('left' or 'right')
            control_color: Target color value
            kp, ki, kd: PID parameters
            slow_start: Whether to start slowly
            slow_down: Whether to slow down at end
            slow_speed_override: Slow speed override
            start_cm_slow: Distance to start slow in cm
        """
        # For line following, we'll approximate as a straight line
        # In reality, the path would depend on the line shape
        distance_cm = distance / 10
        self._update_position_after_drive(distance_cm)
        
        # Create command
        cmd = {
            'type': 'follow_line',
            'distance': distance,
            'speed': speed,
            'edge': edge,
            'control_color': control_color,
            'kp': kp,
            'ki': ki,
            'kd': kd,
            'slow_start': slow_start,
            'slow_down': slow_down,
            'slow_speed_override': slow_speed_override,
            'start_cm_slow': start_cm_slow
        }
        
        self.commands.append(cmd)
        
    def set_pickers_position(self, position, wait=True, speed=700):
        """
        Add a picker position command to the path.
        
        Args:
            position: Position constant (ALL_UP, BELOW_LID, etc.)
            wait: Whether to wait for completion
            speed: Motor speed
        """
        # This doesn't change robot position, just add to commands
        cmd = {
            'type': 'set_pickers_position',
            'position': position,
            'wait': wait,
            'speed': speed
        }
        
        self.commands.append(cmd)
        
    def set_ball_picker_position(self, position, wait=True, speed=1000):
        """
        Add a ball picker position command to the path.
        
        Args:
            position: Position constant (PICK_BALLS, DROP_BALLS, etc.)
            wait: Whether to wait for completion
            speed: Motor speed
        """
        # This doesn't change robot position, just add to commands
        cmd = {
            'type': 'set_ball_picker_position',
            'position': position,
            'wait': wait,
            'speed': speed
        }
        
        self.commands.append(cmd)
        
    def wait(self, milliseconds):
        """
        Add a wait command to the path.
        
        Args:
            milliseconds: Time to wait in milliseconds
        """
        cmd = {
            'type': 'wait',
            'milliseconds': milliseconds
        }
        
        self.commands.append(cmd)
        
    def visualize_path(self, show_grid=True, show_robot=True, show_commands=True, 
                      field_image=None, save_path=None):
        """
        Visualize the robot path.
        
        Args:
            show_grid: Whether to show grid
            show_robot: Whether to show robot at each position
            show_commands: Whether to show command numbers
            field_image: Optional field image path
            save_path: Optional path to save the plot
        """
        fig, ax = plt.subplots(figsize=(15, 10))
        
        # Set up the plot
        ax.set_aspect('equal')
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_title('Robot Path Visualization')
        
        # Add field image if provided
        if field_image:
            try:
                img = plt.imread(field_image)
                ax.imshow(img, extent=(0, self.field_width, 0, self.field_height), alpha=0.3)
            except:
                print(f"Warning: Could not load field image {field_image}")
        
        # Draw field boundaries
        field_rect = patches.Rectangle((0, 0), self.field_width, self.field_height, 
                                     linewidth=2, edgecolor='black', facecolor='none')
        ax.add_patch(field_rect)
        
        # Draw grid
        if show_grid:
            for x in range(0, int(self.field_width) + 1, int(self.grid_size * 10)):
                ax.axvline(x=x, color='gray', alpha=0.3, linewidth=0.5)
            for y in range(0, int(self.field_height) + 1, int(self.grid_size * 10)):
                ax.axhline(y=y, color='gray', alpha=0.3, linewidth=0.5)
        
        # Draw path
        if len(self.path_history) > 1:
            x_coords = [pos[0] for pos in self.path_history]
            y_coords = [pos[1] for pos in self.path_history]
            ax.plot(x_coords, y_coords, 'b-', linewidth=2, alpha=0.7, label='Robot Path')
            
            # Draw arrows to show direction
            for i in range(0, len(self.path_history) - 1, max(1, len(self.path_history) // 20)):
                x, y, heading = self.path_history[i]
                dx = math.cos(math.radians(heading)) * 5
                dy = math.sin(math.radians(heading)) * 5
                ax.arrow(x, y, dx, dy, head_width=2, head_length=2, fc='blue', ec='blue', alpha=0.7)
        
        # Draw robot positions
        if show_robot:
            for i, (x, y, heading) in enumerate(self.path_history):
                if i % max(1, len(self.path_history) // 10) == 0:  # Show every 10th position
                    # Draw robot rectangle
                    robot_rect = patches.Rectangle(
                        (x - self.robot_width/2, y - self.robot_length/2),
                        self.robot_width, self.robot_length,
                        angle=heading,
                        linewidth=1, edgecolor='red', facecolor='red', alpha=0.3
                    )
                    ax.add_patch(robot_rect)
                    
                    # Draw heading indicator
                    dx = math.cos(math.radians(heading)) * (self.robot_width/2 + 2)
                    dy = math.sin(math.radians(heading)) * (self.robot_width/2 + 2)
                    ax.arrow(x, y, dx, dy, head_width=1, head_length=1, fc='red', ec='red')
        
        # Draw command numbers
        if show_commands and self.commands:
            current_x, current_y = self.x, self.y
            for i, cmd in enumerate(self.commands):
                if cmd['type'] in ['drive_straight', 'turn_to_angle', 'curve', 'follow_line']:
                    # Find the position for this command
                    if i < len(self.path_history):
                        x, y, _ = self.path_history[i]
                        ax.annotate(f'{i+1}', (x, y), xytext=(5, 5), 
                                  textcoords='offset points', fontsize=8,
                                  bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
        
        # Set plot limits
        margin = 20
        ax.set_xlim(-margin, self.field_width + margin)
        ax.set_ylim(-margin, self.field_height + margin)
        
        # Add legend
        ax.legend()
        
        # Add statistics
        total_distance = self._calculate_total_distance()
        total_time = self._estimate_total_time()
        stats_text = f'Total Distance: {total_distance:.1f} cm\n'
        stats_text += f'Estimated Time: {total_time:.1f} s\n'
        stats_text += f'Commands: {len(self.commands)}'
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=10,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        plt.show()
        
    def _calculate_total_distance(self):
        """Calculate total distance traveled."""
        total = 0
        for i in range(1, len(self.path_history)):
            x1, y1, _ = self.path_history[i-1]
            x2, y2, _ = self.path_history[i]
            total += math.sqrt((x2-x1)**2 + (y2-y1)**2)
        return total
        
    def _estimate_total_time(self):
        """Estimate total execution time."""
        total_time = 0
        for cmd in self.commands:
            if cmd['type'] == 'drive_straight':
                distance = cmd['distance']
                speed = cmd['speed'] / 10  # Convert mm/s to cm/s
                if speed > 0:
                    total_time += distance / speed
            elif cmd['type'] == 'turn_to_angle':
                # Estimate turn time based on speed
                speed = cmd['speed']
                total_time += 1.0  # Rough estimate
            elif cmd['type'] == 'curve':
                # Estimate curve time
                speed = cmd['speed'] / 10
                if speed > 0:
                    total_time += 2.0  # Rough estimate
            elif cmd['type'] == 'awaitarc':
                # Estimate awaitarc time
                speed = cmd['speed'] / 10
                if speed > 0:
                    if cmd['angle'] is not None:
                        # Estimate based on angle
                        total_time += abs(cmd['angle']) / 90.0  # Rough estimate: 90 degrees = 1 second
                    else:
                        # Estimate based on distance
                        total_time += abs(cmd['distance']) / speed / 10  # Convert mm to cm
            elif cmd['type'] == 'wait':
                total_time += cmd['milliseconds'] / 1000
        return total_time
        
    def generate_code(self, function_name="generated_path"):
        """
        Generate Python code for the path.
        
        Args:
            function_name: Name of the generated function
            
        Returns:
            String containing the generated code
        """
        code = f"""def {function_name}():
    \"\"\"
    Generated path function.
    Total distance: {self._calculate_total_distance():.1f} cm
    Estimated time: {self._estimate_total_time():.1f} seconds
    \"\"\"
"""
        
        # Add imports if needed
        imports_needed = set()
        for cmd in self.commands:
            if cmd['type'] == 'set_pickers_position':
                imports_needed.add('ALL_UP, BELOW_LID, HALF_LIFT_LID, LIFT_LID, DROP_BLOCKS, DROP_BLOCKS_LOWER, PICK_RED, PICK_WHITE, PICK_GREEN, PICK_YELLOW, HOLD_DRONE, GROUND, MIDDLE_DROP_BLOCKS')
            elif cmd['type'] == 'set_ball_picker_position':
                imports_needed.add('PICK_BALLS, DROP_BALLS, SLIDE_BALL, ROVER')
            elif cmd['type'] in ['drive_straight', 'turn_to_angle', 'curve', 'awaitarc', 'follow_line']:
                imports_needed.add('Stop')
                
        if imports_needed:
            code += "    # Import required constants\n"
            for imports in imports_needed:
                code += f"    from utils import {imports}\n"
            code += "\n"
        
        # Generate commands
        for i, cmd in enumerate(self.commands):
            code += f"    # Command {i+1}: {cmd['type']}\n"
            
            if cmd['type'] == 'drive_straight':
                code += f"    gyroStraightWithDrive("
                code += f"distance={cmd['distance']}, speed={cmd['speed']}"
                if cmd['backward']:
                    code += ", backward=True"
                if cmd['target_angle'] is not None:
                    code += f", targetAngle={cmd['target_angle']}"
                if cmd['till_black_line']:
                    code += ", tillBlackLine=True"
                if cmd['till_white_line']:
                    code += ", tillWhiteLine=True"
                if cmd['detect_stall']:
                    code += ", detectStall=True"
                if cmd['stop_when_load_above'] > 0:
                    code += f", stopWhenLoadAbove={cmd['stop_when_load_above']}"
                if not cmd['slow_down']:
                    code += ", slowDown=False"
                if cmd['slow_speed_override'] != 50:
                    code += f", slowSpeedOverride={cmd['slow_speed_override']}"
                code += ")\n"
                
            elif cmd['type'] == 'turn_to_angle':
                code += f"    turnToAngle("
                code += f"targetAngle={cmd['target_angle']}, speed={cmd['speed']}"
                if cmd['force_turn']:
                    code += f", forceTurn=FORCETURN_{cmd['force_turn'].upper()}"
                if cmd['one_wheel_turn']:
                    code += ", oneWheelTurn=True"
                code += ")\n"
                
            elif cmd['type'] == 'curve':
                code += f"    curve("
                code += f"radius={cmd['radius']}, angle={cmd['angle']}, speed={cmd['speed']}"
                if cmd['acceleration'] != 450:
                    code += f", acceleration={cmd['acceleration']}"
                if cmd['deceleration'] != 0:
                    code += f", deceleration={cmd['deceleration']}"
                if cmd['dont_accelerate']:
                    code += ", dont_accelerate=True"
                if cmd['dont_decelerate']:
                    code += ", dont_decelerate=True"
                code += ")\n"
                
            elif cmd['type'] == 'awaitarc':
                code += f"    awaitarc("
                code += f"radius={cmd['radius']}, speed={cmd['speed']}"
                if cmd['angle'] is not None:
                    code += f", angle={cmd['angle']}"
                if cmd['distance'] is not None:
                    code += f", distance={cmd['distance']}"
                if cmd['then'] != "HOLD":
                    code += f", then=Stop.{cmd['then']}"
                if not cmd['wait']:
                    code += ", wait=False"
                code += ")\n"
                
            elif cmd['type'] == 'follow_line':
                code += f"    followBlackLinePID("
                code += f"distanceInMM={cmd['distance']}, speed={cmd['speed']}"
                if cmd['edge'] == 'left':
                    code += ", edge=LINE_FOLLOWER_EDGE_LEFT"
                else:
                    code += ", edge=LINE_FOLLOWER_EDGE_RIGHT"
                if cmd['control_color'] != 63:
                    code += f", controlColor={cmd['control_color']}"
                if cmd['kp'] != 0.5:
                    code += f", kp={cmd['kp']}"
                if cmd['ki'] != 0:
                    code += f", ki={cmd['ki']}"
                if cmd['kd'] != 0:
                    code += f", kd={cmd['kd']}"
                if not cmd['slow_start']:
                    code += ", slowStart=False"
                if not cmd['slow_down']:
                    code += ", slowDown=False"
                if cmd['slow_speed_override'] != 100:
                    code += f", slowSpeedOverride={cmd['slow_speed_override']}"
                if cmd['start_cm_slow'] != 0:
                    code += f", startcmSLOW={cmd['start_cm_slow']}"
                code += ")\n"
                
            elif cmd['type'] == 'set_pickers_position':
                code += f"    setPickersPosition("
                code += f"position={cmd['position']}"
                if not cmd['wait']:
                    code += ", wait=False"
                if cmd['speed'] != 700:
                    code += f", speed={cmd['speed']}"
                code += ")\n"
                
            elif cmd['type'] == 'set_ball_picker_position':
                code += f"    setBallPickerPosition("
                code += f"position={cmd['position']}"
                if not cmd['wait']:
                    code += ", wait=False"
                if cmd['speed'] != 1000:
                    code += f", speed={cmd['speed']}"
                code += ")\n"
                
            elif cmd['type'] == 'wait':
                code += f"    wait({cmd['milliseconds']})\n"
                
            code += "\n"
            
        return code
        
    def save_path(self, filename):
        """Save path to JSON file."""
        data = {
            'robot_width': self.robot_width,
            'robot_length': self.robot_length,
            'initial_position': [self.x, self.y, self.heading],
            'path_history': self.path_history,
            'commands': self.commands
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
            
    def load_path(self, filename):
        """Load path from JSON file."""
        with open(filename, 'r') as f:
            data = json.load(f)
            
        self.robot_width = data['robot_width']
        self.robot_length = data['robot_length']
        self.x, self.y, self.heading = data['initial_position']
        self.path_history = data['path_history']
        self.commands = data['commands']
        
    def get_path_summary(self):
        """Get a summary of the path."""
        total_distance = self._calculate_total_distance()
        total_time = self._estimate_total_time()
        
        summary = {
            'total_distance_cm': total_distance,
            'estimated_time_seconds': total_time,
            'number_of_commands': len(self.commands),
            'final_position': (self.x, self.y, self.heading),
            'command_types': {}
        }
        
        for cmd in self.commands:
            cmd_type = cmd['type']
            summary['command_types'][cmd_type] = summary['command_types'].get(cmd_type, 0) + 1
            
        return summary

    def bezier_curve(self, start, control, end, speed=300, acceleration=450):
        """Add a Bezier curve command to the path."""
        points = self._bezier_curve_points(start, control, end, num=50)
        for i, (x, y) in enumerate(points[1:], 1):
            self.x = x
            self.y = y
            t = i / (len(points) - 1)
            # Calculate tangent vector (derivative of Bezier curve)
            dx = 2*(1-t)*(control[0]-start[0]) + 2*t*(end[0]-control[0])
            dy = 2*(1-t)*(control[1]-start[1]) + 2*t*(end[1]-control[1])
            if dx != 0 or dy != 0:
                # Use positive atan2 for correct direction
                self.heading = math.degrees(math.atan2(dy, dx))
            self._add_to_history()
        cmd = {
            'type': 'bezier_curve',
            'start': start,
            'control': control,
            'end': end,
            'speed': speed,
            'acceleration': acceleration
        }
        self.commands.append(cmd)

    def _bezier_curve_points(self, p0, p1, p2, num=50):
        """Return a list of points along a quadratic Bezier curve."""
        points = []
        for t in np.linspace(0, 1, num):
            x = (1-t)**2 * p0[0] + 2*(1-t)*t*p1[0] + t**2*p2[0]
            y = (1-t)**2 * p0[1] + 2*(1-t)*t*p1[1] + t**2*p2[1]
            points.append((x, y))
        return points

    def interactive_waypoint_editor(self):
        """
        Open an interactive GUI for editing waypoints and generating code.
        """
        editor = WaypointEditor(self)
        editor.run()

    def get_waypoints(self):
        """Extract waypoints from path history."""
        waypoints = []
        for i, (x, y, heading) in enumerate(self.path_history):
            waypoints.append({
                'index': i,
                'x': x,
                'y': y,
                'heading': heading,
                'command': self.commands[i] if i < len(self.commands) else None
            })
        return waypoints

    def update_from_waypoints(self, waypoints):
        """Update path from edited waypoints."""
        self.path_history = [(wp['x'], wp['y'], wp['heading']) for wp in waypoints]
        if waypoints:
            self.x = waypoints[-1]['x']
            self.y = waypoints[-1]['y']
            self.heading = waypoints[-1]['heading']


class WaypointEditor:
    """
    Interactive GUI for editing waypoints and generating robot code.
    """
    
    def __init__(self, path_planner):
        self.path_planner = path_planner
        self.waypoints = path_planner.get_waypoints()
        self.selected_waypoint = None
        
        # Drag state variables
        self.dragging = False
        self.drag_start = None
        self.drag_waypoint = None
        
        # Robot visualization
        self.robot_size = 15  # Robot size in cm
        self.show_robot = True  # Whether to show robot
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("Path Planner - Interactive Waypoint Editor")
        self.root.geometry("1400x900")
        
        # Create GUI elements
        self.setup_ui()
        self.update_plot()
        
    def setup_ui(self):
        """Setup the user interface."""
        # Create main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create left panel for controls with scrolling
        left_panel = ttk.Frame(main_frame, width=300)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # Create canvas and scrollbar for left panel
        left_canvas = tk.Canvas(left_panel, width=300)
        left_scrollbar = ttk.Scrollbar(left_panel, orient="vertical", command=left_canvas.yview)
        scrollable_frame = ttk.Frame(left_canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: left_canvas.configure(scrollregion=left_canvas.bbox("all"))
        )
        
        left_canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        left_canvas.configure(yscrollcommand=left_scrollbar.set)
        
        # Pack the canvas and scrollbar
        left_canvas.pack(side="left", fill="both", expand=True)
        left_scrollbar.pack(side="right", fill="y")
        
        # Create right panel for plot
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.setup_controls(scrollable_frame)
        self.setup_plot_area(right_panel)
        
    def setup_controls(self, parent):
        """Setup the control panel."""
        # Title
        title_label = ttk.Label(parent, text="Waypoint Editor Controls", font=("Arial", 14, "bold"))
        title_label.pack(pady=(0, 20))
        
        # Waypoint list
        waypoint_frame = ttk.LabelFrame(parent, text="Waypoints", padding=10)
        waypoint_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Create listbox with scrollbar
        list_frame = ttk.Frame(waypoint_frame)
        list_frame.pack(fill=tk.BOTH, expand=True)
        
        self.waypoint_listbox = tk.Listbox(list_frame, height=8)
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.waypoint_listbox.yview)
        self.waypoint_listbox.configure(yscrollcommand=scrollbar.set)
        
        self.waypoint_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Bind selection event
        self.waypoint_listbox.bind('<<ListboxSelect>>', self.on_waypoint_select)
        
        # Waypoint actions
        actions_frame = ttk.LabelFrame(parent, text="Waypoint Actions", padding=10)
        actions_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(actions_frame, text="Add Waypoint", command=self.add_waypoint).pack(fill=tk.X, pady=2)
        ttk.Button(actions_frame, text="Edit Waypoint", command=self.edit_waypoint).pack(fill=tk.X, pady=2)
        ttk.Button(actions_frame, text="Delete Waypoint", command=self.delete_waypoint).pack(fill=tk.X, pady=2)
        
        # Path actions
        path_frame = ttk.LabelFrame(parent, text="Path Actions", padding=10)
        path_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(path_frame, text="Clear Path", command=self.clear_path).pack(fill=tk.X, pady=2)
        ttk.Button(path_frame, text="Undo Last", command=self.undo_last).pack(fill=tk.X, pady=2)
        ttk.Button(path_frame, text="Generate Code", command=self.generate_code).pack(fill=tk.X, pady=2)
        ttk.Button(path_frame, text="Save Path", command=self.save_path).pack(fill=tk.X, pady=2)
        ttk.Button(path_frame, text="Load Path", command=self.load_path).pack(fill=tk.X, pady=2)
        
        # Path info
        info_frame = ttk.LabelFrame(parent, text="Path Information", padding=10)
        info_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.info_text = tk.Text(info_frame, height=6, width=35)
        self.info_text.pack(fill=tk.BOTH, expand=True)
        
        # Robot visualization control
        robot_frame = ttk.LabelFrame(parent, text="Robot Visualization", padding=10)
        robot_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Robot size control
        ttk.Label(robot_frame, text="Robot Size (cm):").pack(anchor=tk.W)
        self.robot_size_var = tk.StringVar(value="15")
        size_entry = ttk.Entry(robot_frame, textvariable=self.robot_size_var, width=10)
        size_entry.pack(anchor=tk.W, pady=(0, 5))
        size_entry.bind('<KeyRelease>', self.on_robot_size_change)
        
        # Show robot checkbox
        self.show_robot_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(robot_frame, text="Show Robot", 
                       variable=self.show_robot_var, command=self.on_show_robot_change).pack(anchor=tk.W)
        
        # Instructions
        instructions_frame = ttk.LabelFrame(parent, text="Instructions", padding=10)
        instructions_frame.pack(fill=tk.X, pady=(0, 10))
        
        instructions_text = """
• Click and drag waypoints to move them
• Click and drag empty space to add new waypoints
• Right-click for context menus
• Use the list to select waypoints
• Watch the status bar for feedback
        """
        ttk.Label(instructions_frame, text=instructions_text, justify=tk.LEFT).pack(anchor=tk.W)
        
        # Update waypoint list and info
        self.update_waypoint_list()
        self.update_info()
        
        # Status bar
        status_frame = ttk.Frame(self.root)
        status_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=(0, 10))
        
        self.status_label = ttk.Label(status_frame, text="Ready - Click and drag waypoints to edit")
        self.status_label.pack(side=tk.LEFT)
        
    def setup_plot_area(self, parent):
        """Setup the plot area."""
        # Create plot
        self.fig = Figure(figsize=(10, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add navigation toolbar
        from matplotlib.backends._backend_tk import NavigationToolbar2Tk
        toolbar = NavigationToolbar2Tk(self.canvas, parent)
        toolbar.update()
        
        # Bind mouse events for drag functionality
        self.canvas.mpl_connect('button_press_event', self.on_mouse_press)
        self.canvas.mpl_connect('button_release_event', self.on_mouse_release)
        self.canvas.mpl_connect('motion_notify_event', self.on_mouse_motion)
        self.canvas.mpl_connect('button_press_event', self.on_right_click)
        
    def update_waypoint_list(self):
        """Update the waypoint listbox."""
        self.waypoint_listbox.delete(0, tk.END)
        for i, wp in enumerate(self.waypoints):
            display_text = f"WP {i}: ({wp['x']:.1f}, {wp['y']:.1f}) @ {wp['heading']:.1f}°"
            if wp['command']:
                cmd_type = wp['command']['type']
                display_text += f" - {cmd_type}"
            self.waypoint_listbox.insert(tk.END, display_text)
            
    def on_waypoint_select(self, event):
        """Handle waypoint selection."""
        selection = self.waypoint_listbox.curselection()
        if selection:
            self.selected_waypoint = selection[0]
        else:
            self.selected_waypoint = None
            
    def add_waypoint(self):
        """Add a new waypoint."""
        dialog = AddWaypointDialog(self.root)
        if dialog.result:
            x, y, heading = dialog.result
            new_waypoint = {
                'index': len(self.waypoints),
                'x': x,
                'y': y,
                'heading': heading,
                'command': None
            }
            self.waypoints.append(new_waypoint)
            self.update_waypoint_list()
            self.update_plot()
            self.update_info()
            
    def edit_waypoint(self):
        """Edit the selected waypoint."""
        if self.selected_waypoint is None:
            messagebox.showwarning("Warning", "Please select a waypoint to edit.")
            return
            
        wp = self.waypoints[self.selected_waypoint]
        dialog = EditWaypointDialog(self.root, wp)
        if dialog.result:
            x, y, heading = dialog.result
            wp['x'] = x
            wp['y'] = y
            wp['heading'] = heading
            self.update_waypoint_list()
            self.update_plot()
            self.update_info()
            
    def delete_waypoint(self):
        """Delete the selected waypoint."""
        if self.selected_waypoint is None:
            messagebox.showwarning("Warning", "Please select a waypoint to delete.")
            return
            
        if messagebox.askyesno("Confirm Delete", "Are you sure you want to delete this waypoint?"):
            del self.waypoints[self.selected_waypoint]
            # Update indices
            for i, wp in enumerate(self.waypoints):
                wp['index'] = i
            self.update_waypoint_list()
            self.update_plot()
            self.update_info()
            
    def generate_code(self):
        """Generate code from current waypoints."""
        # Update path planner with current waypoints
        self.path_planner.update_from_waypoints(self.waypoints)
        
        # Generate code
        code = self.path_planner.generate_code("edited_path")
        
        # Show code in dialog
        CodeDisplayDialog(self.root, code)
        
    def save_path(self):
        """Save the current path."""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            self.path_planner.update_from_waypoints(self.waypoints)
            self.path_planner.save_path(filename)
            messagebox.showinfo("Success", f"Path saved to {filename}")
            
    def load_path(self):
        """Load a path from file."""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            try:
                self.path_planner.load_path(filename)
                self.waypoints = self.path_planner.get_waypoints()
                self.update_waypoint_list()
                self.update_plot()
                self.update_info()
                messagebox.showinfo("Success", f"Path loaded from {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load path: {str(e)}")
                
    def update_plot(self):
        """Update the path visualization plot."""
        self.fig.clear()
        ax = self.fig.add_subplot(111)
        
        # Set up the plot
        ax.set_aspect('equal')
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_title('Interactive Path Editor')
        
        # Draw field boundaries
        field_rect = patches.Rectangle((0, 0), self.path_planner.field_width, self.path_planner.field_height, 
                                     linewidth=2, edgecolor='black', facecolor='none')
        ax.add_patch(field_rect)
        
        # Draw grid
        for x in range(0, int(self.path_planner.field_width) + 1, 10):
            ax.axvline(x=x, color='gray', alpha=0.3, linewidth=0.5)
        for y in range(0, int(self.path_planner.field_height) + 1, 10):
            ax.axhline(y=y, color='gray', alpha=0.3, linewidth=0.5)
        
        # Draw waypoints and path
        if len(self.waypoints) > 1:
            x_coords = [wp['x'] for wp in self.waypoints]
            y_coords = [wp['y'] for wp in self.waypoints]
            ax.plot(x_coords, y_coords, 'b-', linewidth=2, alpha=0.7, label='Robot Path')
            
            # Draw arrows to show direction
            for i, wp in enumerate(self.waypoints[:-1]):
                x, y, heading = wp['x'], wp['y'], wp['heading']
                dx = math.cos(math.radians(heading)) * 5
                dy = math.sin(math.radians(heading)) * 5
                ax.arrow(x, y, dx, dy, head_width=2, head_length=2, fc='blue', ec='blue', alpha=0.7)
        
        # Draw waypoints
        for i, wp in enumerate(self.waypoints):
            x, y, heading = wp['x'], wp['y'], wp['heading']
            
            # Draw waypoint marker
            color = 'red' if i == self.selected_waypoint else 'green'
            ax.plot(x, y, 'o', color=color, markersize=8, markeredgecolor='black', markeredgewidth=2)
            
            # Draw robot at waypoint if enabled
            if self.show_robot:
                robot_rect = patches.Rectangle(
                    (x - self.robot_size/2, y - self.robot_size/2),
                    self.robot_size, self.robot_size,
                    angle=heading,
                    linewidth=1, edgecolor=color, facecolor=color, alpha=0.3
                )
                ax.add_patch(robot_rect)
                
                # Draw heading indicator
                dx = math.cos(math.radians(heading)) * (self.robot_size/2 + 2)
                dy = math.sin(math.radians(heading)) * (self.robot_size/2 + 2)
                ax.arrow(x, y, dx, dy, head_width=1, head_length=1, fc=color, ec=color)
            
            # Add waypoint number
            ax.annotate(f'{i}', (x, y), xytext=(5, 5), 
                       textcoords='offset points', fontsize=10,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
        
        # Set plot limits
        margin = 20
        ax.set_xlim(-margin, self.path_planner.field_width + margin)
        ax.set_ylim(-margin, self.path_planner.field_height + margin)
        
        # Add legend
        ax.legend()
        
        # Add statistics
        total_distance = self.calculate_total_distance()
        stats_text = f'Total Distance: {total_distance:.1f} cm\n'
        stats_text += f'Waypoints: {len(self.waypoints)}'
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=10,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Add instructions
        instructions = "Click and drag waypoints to move them • Click and drag empty space to add new waypoints"
        ax.text(0.02, 0.02, instructions, transform=ax.transAxes, fontsize=9,
                verticalalignment='bottom', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        self.canvas.draw()
        
    def calculate_total_distance(self):
        """Calculate total distance between waypoints."""
        total = 0
        for i in range(1, len(self.waypoints)):
            x1, y1 = self.waypoints[i-1]['x'], self.waypoints[i-1]['y']
            x2, y2 = self.waypoints[i]['x'], self.waypoints[i]['y']
            total += math.sqrt((x2-x1)**2 + (y2-y1)**2)
        return total
    
    def on_mouse_press(self, event):
        """Handle mouse press events for drag functionality."""
        if event.inaxes != self.fig.axes[0]:
            return
            
        # Check if click is near any waypoint
        waypoint_found = False
        for i, wp in enumerate(self.waypoints):
            x, y = wp['x'], wp['y']
            # Check if click is within 10 pixels of waypoint
            if abs(event.xdata - x) < 10 and abs(event.ydata - y) < 10:
                self.dragging = True
                self.drag_start = (event.xdata, event.ydata)
                self.drag_waypoint = i
                self.selected_waypoint = i
                waypoint_found = True
                self.status_label.config(text=f"Dragging waypoint {i} - Release to set new position")
                self.update_waypoint_list()
                self.update_plot()
                self.update_info()
                break
        
        # If no waypoint was clicked, start creating a new one
        if not waypoint_found and event.button == 1:  # Left click
            self.dragging = True
            self.drag_start = (event.xdata, event.ydata)
            self.drag_waypoint = None  # Will create new waypoint on release
            self.status_label.config(text="Creating new waypoint - Drag to set position and direction")
    
    def on_mouse_release(self, event):
        """Handle mouse release events for drag functionality."""
        if self.dragging:
            if event.inaxes == self.fig.axes[0]:
                new_x = max(0, min(self.path_planner.field_width, event.xdata))
                new_y = max(0, min(self.path_planner.field_height, event.ydata))
                
                if self.drag_waypoint is not None:
                    # Update existing waypoint position
                    self.waypoints[self.drag_waypoint]['x'] = new_x
                    self.waypoints[self.drag_waypoint]['y'] = new_y
                else:
                    # Create new waypoint
                    # Calculate heading based on drag direction
                    if self.drag_start:
                        dx = new_x - self.drag_start[0]
                        dy = new_y - self.drag_start[1]
                        if abs(dx) > 0.1 or abs(dy) > 0.1:  # Only if there's significant movement
                            heading = math.degrees(math.atan2(dy, dx))
                        else:
                            heading = 0
                    else:
                        heading = 0
                    
                    new_waypoint = {
                        'index': len(self.waypoints),
                        'x': new_x,
                        'y': new_y,
                        'heading': heading,
                        'command': None
                    }
                    self.waypoints.append(new_waypoint)
                    self.selected_waypoint = len(self.waypoints) - 1
            
            self.dragging = False
            self.drag_start = None
            self.drag_waypoint = None
            self.status_label.config(text="Ready - Click and drag waypoints to edit")
            self.update_waypoint_list()
            self.update_plot()
            self.update_info()
    
    def on_mouse_motion(self, event):
        """Handle mouse motion events for drag functionality."""
        if self.dragging and event.inaxes == self.fig.axes[0]:
            if self.drag_waypoint is not None:
                # Update existing waypoint position in real-time during drag
                new_x = max(0, min(self.path_planner.field_width, event.xdata))
                new_y = max(0, min(self.path_planner.field_height, event.ydata))
                self.waypoints[self.drag_waypoint]['x'] = new_x
                self.waypoints[self.drag_waypoint]['y'] = new_y
                
                # Update status with current position
                self.status_label.config(text=f"Dragging waypoint {self.drag_waypoint} to ({new_x:.1f}, {new_y:.1f})")
                
                # Update plot in real-time
                self.update_plot()
            else:
                # Show preview of new waypoint being created
                new_x = max(0, min(self.path_planner.field_width, event.xdata))
                new_y = max(0, min(self.path_planner.field_height, event.ydata))
                
                # Calculate heading based on drag direction
                if self.drag_start:
                    dx = new_x - self.drag_start[0]
                    dy = new_y - self.drag_start[1]
                    if abs(dx) > 0.1 or abs(dy) > 0.1:
                        heading = math.degrees(math.atan2(dy, dx))
                    else:
                        heading = 0
                else:
                    heading = 0
                
                self.status_label.config(text=f"Creating waypoint at ({new_x:.1f}, {new_y:.1f}) heading {heading:.1f}°")
                self.update_plot()
        elif event.inaxes == self.fig.axes[0]:
            # Show mouse position when not dragging
            self.status_label.config(text=f"Mouse at ({event.xdata:.1f}, {event.ydata:.1f}) - Click and drag to edit")
    
    def on_right_click(self, event):
        """Handle right-click events for context menu."""
        if event.button == 3 and event.inaxes == self.fig.axes[0]:  # Right click
            # Find waypoint under cursor
            clicked_waypoint = None
            for i, wp in enumerate(self.waypoints):
                x, y = wp['x'], wp['y']
                if abs(event.xdata - x) < 10 and abs(event.ydata - y) < 10:
                    clicked_waypoint = i
                    break
            
            if clicked_waypoint is not None:
                # Show context menu for waypoint
                self.show_waypoint_context_menu(event, clicked_waypoint)
            else:
                # Show context menu for empty space
                self.show_empty_space_context_menu(event)
    
    def show_waypoint_context_menu(self, event, waypoint_index):
        """Show context menu for a waypoint."""
        menu = tk.Menu(self.root, tearoff=0)
        menu.add_command(label=f"Edit Waypoint {waypoint_index}", 
                        command=lambda: self.edit_waypoint_at_index(waypoint_index))
        menu.add_command(label=f"Delete Waypoint {waypoint_index}", 
                        command=lambda: self.delete_waypoint_at_index(waypoint_index))
        menu.add_separator()
        menu.add_command(label="Insert Waypoint Before", 
                        command=lambda: self.insert_waypoint_before(waypoint_index, event.xdata, event.ydata))
        menu.add_command(label="Insert Waypoint After", 
                        command=lambda: self.insert_waypoint_after(waypoint_index, event.xdata, event.ydata))
        
        # Convert matplotlib coordinates to screen coordinates
        x_screen = self.root.winfo_rootx() + self.canvas.get_tk_widget().winfo_x() + event.x
        y_screen = self.root.winfo_rooty() + self.canvas.get_tk_widget().winfo_y() + event.y
        menu.post(x_screen, y_screen)
    
    def show_empty_space_context_menu(self, event):
        """Show context menu for empty space."""
        menu = tk.Menu(self.root, tearoff=0)
        menu.add_command(label="Add Waypoint Here", 
                        command=lambda: self.add_waypoint_at_position(event.xdata, event.ydata))
        menu.add_command(label="Add Waypoint with Dialog", 
                        command=self.add_waypoint)
        
        # Convert matplotlib coordinates to screen coordinates
        x_screen = self.root.winfo_rootx() + self.canvas.get_tk_widget().winfo_x() + event.x
        y_screen = self.root.winfo_rooty() + self.canvas.get_tk_widget().winfo_y() + event.y
        menu.post(x_screen, y_screen)
    
    def edit_waypoint_at_index(self, index):
        """Edit waypoint at specific index."""
        self.selected_waypoint = index
        self.edit_waypoint()
    
    def delete_waypoint_at_index(self, index):
        """Delete waypoint at specific index."""
        self.selected_waypoint = index
        self.delete_waypoint()
    
    def insert_waypoint_before(self, index, x, y):
        """Insert a new waypoint before the specified index."""
        new_waypoint = {
            'index': index,
            'x': x,
            'y': y,
            'heading': 0,
            'command': None
        }
        self.waypoints.insert(index, new_waypoint)
        # Update indices
        for i in range(index + 1, len(self.waypoints)):
            self.waypoints[i]['index'] = i
        self.selected_waypoint = index
        self.update_waypoint_list()
        self.update_plot()
        self.update_info()
    
    def insert_waypoint_after(self, index, x, y):
        """Insert a new waypoint after the specified index."""
        new_waypoint = {
            'index': index + 1,
            'x': x,
            'y': y,
            'heading': 0,
            'command': None
        }
        self.waypoints.insert(index + 1, new_waypoint)
        # Update indices
        for i in range(index + 2, len(self.waypoints)):
            self.waypoints[i]['index'] = i
        self.selected_waypoint = index + 1
        self.update_waypoint_list()
        self.update_plot()
        self.update_info()
    
    def add_waypoint_at_position(self, x, y):
        """Add a new waypoint at the specified position."""
        new_waypoint = {
            'index': len(self.waypoints),
            'x': x,
            'y': y,
            'heading': 0,
            'command': None
        }
        self.waypoints.append(new_waypoint)
        self.selected_waypoint = len(self.waypoints) - 1
        self.update_waypoint_list()
        self.update_plot()
        self.update_info()
    
    def clear_path(self):
        """Clear all waypoints."""
        if messagebox.askyesno("Confirm Clear", "Are you sure you want to clear all waypoints?"):
            self.waypoints = []
            self.selected_waypoint = None
            self.update_waypoint_list()
            self.update_plot()
            self.update_info()
    
    def undo_last(self):
        """Remove the last waypoint."""
        if self.waypoints:
            del self.waypoints[-1]
            if self.selected_waypoint == len(self.waypoints):
                self.selected_waypoint = None
            self.update_waypoint_list()
            self.update_plot()
            self.update_info()
    
    def update_info(self):
        """Update the path information display."""
        total_distance = self.calculate_total_distance()
        info = f"Total Distance: {total_distance:.1f} cm\n"
        info += f"Number of Waypoints: {len(self.waypoints)}\n"
        if self.waypoints:
            info += f"Last Position: ({self.waypoints[-1]['x']:.1f}, {self.waypoints[-1]['y']:.1f})\n"
            info += f"Last Heading: {self.waypoints[-1]['heading']:.1f}°\n"
        else:
            info += "No waypoints\n"
        
        self.info_text.delete(1.0, tk.END)
        self.info_text.insert(1.0, info)
    
    def on_robot_size_change(self, event):
        """Handle robot size change."""
        try:
            self.robot_size = float(self.robot_size_var.get())
            self.update_plot()
        except ValueError:
            pass
    
    def on_show_robot_change(self):
        """Handle show robot checkbox change."""
        self.show_robot = self.show_robot_var.get()
        self.update_plot()
        
    def run(self):
        """Start the GUI event loop."""
        self.root.mainloop()


class AddWaypointDialog:
    """Dialog for adding a new waypoint."""
    
    def __init__(self, parent):
        self.result = None
        
        # Create dialog
        self.dialog = tk.Toplevel(parent)
        self.dialog.title("Add Waypoint")
        self.dialog.geometry("300x200")
        self.dialog.transient(parent)
        self.dialog.grab_set()
        
        # Center dialog
        self.dialog.geometry("+%d+%d" % (parent.winfo_rootx() + 50, parent.winfo_rooty() + 50))
        
        # Create widgets
        ttk.Label(self.dialog, text="X coordinate (cm):").pack(pady=5)
        self.x_entry = ttk.Entry(self.dialog)
        self.x_entry.pack(pady=5)
        self.x_entry.insert(0, "0")
        
        ttk.Label(self.dialog, text="Y coordinate (cm):").pack(pady=5)
        self.y_entry = ttk.Entry(self.dialog)
        self.y_entry.pack(pady=5)
        self.y_entry.insert(0, "0")
        
        ttk.Label(self.dialog, text="Heading (degrees):").pack(pady=5)
        self.heading_entry = ttk.Entry(self.dialog)
        self.heading_entry.pack(pady=5)
        self.heading_entry.insert(0, "0")
        
        # Buttons
        button_frame = ttk.Frame(self.dialog)
        button_frame.pack(pady=20)
        
        ttk.Button(button_frame, text="Add", command=self.add).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=self.cancel).pack(side=tk.RIGHT, padx=5)
        
        # Focus on first entry
        self.x_entry.focus()
        
    def add(self):
        """Add the waypoint."""
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            heading = float(self.heading_entry.get())
            self.result = (x, y, heading)
            self.dialog.destroy()
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers.")
            
    def cancel(self):
        """Cancel the dialog."""
        self.dialog.destroy()


class EditWaypointDialog:
    """Dialog for editing an existing waypoint."""
    
    def __init__(self, parent, waypoint):
        self.result = None
        
        # Create dialog
        self.dialog = tk.Toplevel(parent)
        self.dialog.title("Edit Waypoint")
        self.dialog.geometry("300x200")
        self.dialog.transient(parent)
        self.dialog.grab_set()
        
        # Center dialog
        self.dialog.geometry("+%d+%d" % (parent.winfo_rootx() + 50, parent.winfo_rooty() + 50))
        
        # Create widgets
        ttk.Label(self.dialog, text="X coordinate (cm):").pack(pady=5)
        self.x_entry = ttk.Entry(self.dialog)
        self.x_entry.pack(pady=5)
        self.x_entry.insert(0, str(waypoint['x']))
        
        ttk.Label(self.dialog, text="Y coordinate (cm):").pack(pady=5)
        self.y_entry = ttk.Entry(self.dialog)
        self.y_entry.pack(pady=5)
        self.y_entry.insert(0, str(waypoint['y']))
        
        ttk.Label(self.dialog, text="Heading (degrees):").pack(pady=5)
        self.heading_entry = ttk.Entry(self.dialog)
        self.heading_entry.pack(pady=5)
        self.heading_entry.insert(0, str(waypoint['heading']))
        
        # Buttons
        button_frame = ttk.Frame(self.dialog)
        button_frame.pack(pady=20)
        
        ttk.Button(button_frame, text="Update", command=self.update).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=self.cancel).pack(side=tk.RIGHT, padx=5)
        
        # Focus on first entry
        self.x_entry.focus()
        
    def update(self):
        """Update the waypoint."""
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            heading = float(self.heading_entry.get())
            self.result = (x, y, heading)
            self.dialog.destroy()
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers.")
            
    def cancel(self):
        """Cancel the dialog."""
        self.dialog.destroy()


class CodeDisplayDialog:
    """Dialog for displaying generated code."""
    
    def __init__(self, parent, code):
        # Create dialog
        self.dialog = tk.Toplevel(parent)
        self.dialog.title("Generated Code")
        self.dialog.geometry("800x600")
        self.dialog.transient(parent)
        
        # Center dialog
        self.dialog.geometry("+%d+%d" % (parent.winfo_rootx() + 50, parent.winfo_rooty() + 50))
        
        # Create text widget
        text_frame = ttk.Frame(self.dialog)
        text_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.text_widget = tk.Text(text_frame, wrap=tk.WORD, font=('Courier', 10))
        scrollbar = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=self.text_widget.yview)
        self.text_widget.configure(yscrollcommand=scrollbar.set)
        
        self.text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Insert code
        self.text_widget.insert(tk.END, code)
        self.text_widget.config(state=tk.DISABLED)
        
        # Buttons
        button_frame = ttk.Frame(self.dialog)
        button_frame.pack(pady=10)
        
        ttk.Button(button_frame, text="Copy to Clipboard", command=self.copy_to_clipboard).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Save to File", command=self.save_to_file).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Close", command=self.dialog.destroy).pack(side=tk.RIGHT, padx=5)
        
    def copy_to_clipboard(self):
        """Copy code to clipboard."""
        self.dialog.clipboard_clear()
        self.dialog.clipboard_append(self.text_widget.get(1.0, tk.END))
        messagebox.showinfo("Success", "Code copied to clipboard!")
        
    def save_to_file(self):
        """Save code to file."""
        filename = filedialog.asksaveasfilename(
            defaultextension=".py",
            filetypes=[("Python files", "*.py"), ("All files", "*.*")]
        )
        if filename:
            with open(filename, 'w') as f:
                f.write(self.text_widget.get(1.0, tk.END))
            messagebox.showinfo("Success", f"Code saved to {filename}")


# Example usage and demonstration
def create_example_path():
    """Create an example path to demonstrate the tool."""
    planner = PathPlanner()
    
    # Start at origin
    planner.reset_position(0, 0, 0)
    
    # Drive forward
    planner.drive_straight(50, speed=400)
    
    # Turn right
    planner.turn_to_angle(90, speed=400)
    
    # Drive forward
    planner.drive_straight(30, speed=400)
    
    # Curve left
    planner.curve(100, 90, speed=300)
    
    # Follow line
    planner.follow_line(100, speed=200, edge='left')
    
    # Set picker position
    planner.set_pickers_position('ALL_UP')
    
    # Turn back
    planner.turn_to_angle(180, speed=400)
    
    # Drive back
    planner.drive_straight(80, speed=400, backward=True)
    
    return planner


if __name__ == "__main__":
    # Create example path
    planner = create_example_path()
    
    # Visualize the path
    planner.visualize_path()
    
    # Generate code
    code = planner.generate_code("example_path")
    print("Generated Code:")
    print(code)
    
    # Print summary
    summary = planner.get_path_summary()
    print("\nPath Summary:")
    for key, value in summary.items():
        print(f"{key}: {value}") 