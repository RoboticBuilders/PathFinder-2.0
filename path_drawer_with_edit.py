# -*- coding: utf-8 -*-
"""
Interactive Path Drawer for LEGO SPIKE Prime Robot
A visual tool for drawing robot paths and automatically generating code.
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends._backend_tk import NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.transforms import Affine2D
import numpy as np
import math
from typing import List, Tuple, Dict, Optional
import json
from PIL import Image

from path_planner import PathPlanner

class PathDrawer:
    """
    Interactive path drawing tool with automatic code generation.
    """
    
    def __init__(self, root):
        self.root = root
        self.root.title("Path Drawer - LEGO SPIKE Prime Robot")
        self.root.geometry("1400x900")
        
        # Initialize path planner
        self.planner = PathPlanner()
        # Start position will be set after UI setup
        
        # Drawing state
        self.drawing_mode = "drive"  # drive, turn, curve, line_follow
        self.is_drawing = False
        self.start_point = None
        self.current_path_points = []
        self.path_segments = []
        
        # Turn angle control
        self.turn_angle = 0  # Current turn angle in degrees
        self.angle_step = 5  # Angle change per arrow key press
        
        # Path continuation
        self.continue_path = True  # Whether to continue from last position
        
        # Robot visualization
        self.robot_size = 15  # Robot size in cm
        self.show_robot = True  # Whether to show robot
        self.robot_preview_mode = False  # Whether in preview mode (moving with arrow keys)
        
        # Field and missions
        self.field_image = None
        # WRO 2025 Junior Field dimensions: 236cm x 114cm
        # Field image used as background only
        
        # UI elements
        self.canvas = None
        self.ax = None
        self.fig = None
        
        self.setup_ui()
        self.load_field_image()  # Re-enabled with optimizations
        
        # Set initial position based on selected start position
        x, y, heading = self.get_start_position()
        self.planner.reset_position(x, y, heading)
        
        # Initialize plot after UI setup to avoid None errors
        if self.ax is not None:
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
        title_label = ttk.Label(parent, text="Path Drawer Controls", font=("Arial", 14, "bold"))
        title_label.pack(pady=(0, 20))
        
        # Drawing mode selection
        mode_frame = ttk.LabelFrame(parent, text="Drawing Mode", padding=10)
        mode_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.mode_var = tk.StringVar(value="drive")
        modes = [
            ("Drive Straight", "drive"),
            ("Turn to Angle", "turn"),
            ("Curve", "curve"),
            ("Line Follow", "line_follow"),
            ("Set Pickers", "pickers"),
            ("Set Ball Picker", "ball_picker"),
            ("Wait", "wait")
        ]
        
        for text, mode in modes:
            ttk.Radiobutton(mode_frame, text=text, variable=self.mode_var, 
                           value=mode, command=self.on_mode_change).pack(anchor=tk.W)
        
        # Parameters frame
        self.params_frame = ttk.LabelFrame(parent, text="Parameters", padding=10)
        self.params_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.setup_drive_params()
        
        # Action buttons
        actions_frame = ttk.LabelFrame(parent, text="Actions", padding=10)
        actions_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(actions_frame, text="Clear Path", command=self.clear_path).pack(fill=tk.X, pady=2)
        ttk.Button(actions_frame, text="Undo Last", command=self.undo_last).pack(fill=tk.X, pady=2)
        ttk.Button(actions_frame, text="Generate Code", command=self.generate_code).pack(fill=tk.X, pady=2)
        ttk.Button(actions_frame, text="Save Path", command=self.save_path).pack(fill=tk.X, pady=2)
        ttk.Button(actions_frame, text="Load Path", command=self.load_path).pack(fill=tk.X, pady=2)
        
        # Waypoint editing section
        waypoint_frame = ttk.LabelFrame(parent, text="Waypoint Editing", padding=10)
        waypoint_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(waypoint_frame, text="Extract Waypoints from Code", command=self.extract_waypoints_from_code).pack(fill=tk.X, pady=2)
        ttk.Button(waypoint_frame, text="Edit Waypoints", command=self.edit_waypoints).pack(fill=tk.X, pady=2)
        ttk.Button(waypoint_frame, text="Generate Code from Waypoints", command=self.generate_code_from_waypoints).pack(fill=tk.X, pady=2)
        ttk.Button(waypoint_frame, text="Run Program & Extract Waypoints", command=self.run_program_and_extract).pack(fill=tk.X, pady=2)
        
        # Path info
        info_frame = ttk.LabelFrame(parent, text="Path Information", padding=10)
        info_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.info_text = tk.Text(info_frame, height=8, width=35)
        self.info_text.pack(fill=tk.BOTH, expand=True)
        
        # Turn angle control frame
        angle_frame = ttk.LabelFrame(parent, text="Turn Angle Control", padding=10)
        angle_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(angle_frame, text="Use arrow keys to adjust turn angle:").pack(anchor=tk.W)
        ttk.Label(angle_frame, text="← → : Adjust angle by 5°").pack(anchor=tk.W)
        ttk.Label(angle_frame, text="↑ ↓ : Adjust angle by 15°").pack(anchor=tk.W)
        ttk.Label(angle_frame, text="(In Preview Mode: move/turn robot)").pack(anchor=tk.W)
        
        self.angle_var = tk.StringVar(value="0°")
        angle_display = ttk.Label(angle_frame, textvariable=self.angle_var, font=("Arial", 12, "bold"))
        angle_display.pack(pady=5)
        
        # Add direction indicator
        self.direction_var = tk.StringVar(value="→ Right")
        direction_display = ttk.Label(angle_frame, textvariable=self.direction_var, font=("Arial", 10))
        direction_display.pack(pady=2)
        
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
        
        # Preview mode checkbox
        self.preview_mode_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(robot_frame, text="Preview Mode (Move with Arrow Keys)", 
                       variable=self.preview_mode_var, command=self.on_preview_mode_change).pack(anchor=tk.W)
        
        # Start position control
        start_pos_frame = ttk.LabelFrame(parent, text="Start Position", padding=10)
        start_pos_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(start_pos_frame, text="Select starting position:").pack(anchor=tk.W)
        self.start_pos_var = tk.StringVar(value="bottom_left")
        start_pos_combo = ttk.Combobox(start_pos_frame, textvariable=self.start_pos_var, 
                                      values=["Bottom Left (RobotMission)", "Center Far Side"], 
                                      state="readonly", width=25)
        start_pos_combo.pack(fill=tk.X, pady=(0, 5))
        start_pos_combo.bind('<<ComboboxSelected>>', self.on_start_pos_change)
        
        # Path continuation control
        continue_frame = ttk.LabelFrame(parent, text="Path Continuation", padding=10)
        continue_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.continue_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(continue_frame, text="Continue from last position", 
                       variable=self.continue_var, command=self.on_continue_change).pack(anchor=tk.W)
        
        # Wait for button press checkbox
        self.wait_button_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(continue_frame, text="Wait for right button press after each command", 
                       variable=self.wait_button_var).pack(anchor=tk.W)
        
        ttk.Button(continue_frame, text="Reset to Origin", 
                  command=self.reset_to_origin).pack(fill=tk.X, pady=(5, 0))
        
        # Instructions
        instructions_frame = ttk.LabelFrame(parent, text="Instructions", padding=10)
        instructions_frame.pack(fill=tk.X)
        
        instructions = """
Drawing Instructions:
1. Select drawing mode
2. Set parameters
3. Click and drag to draw
4. For turns: use arrow keys to set angle, then click
5. For curves: set radius and angle, then click to execute
6. For drives: click and drag in any direction
7. Use arrow keys to adjust turn angles
8. Check 'Continue from last position' to build paths
9. Robot starts in bottom left corner (RobotMission box)

Robot Visualization:
• Adjust robot size with the size field
• Toggle robot visibility with checkbox
• Enable Preview Mode to move robot with arrow keys:
  - ↑↓ : Move forward/backward
  - ←→ : Turn left/right
• Blue robot shows current position
• Red robots show path history

Angle System:
• 0° = pointing right (+X direction)
• -90° = pointing up (toward top wall)
• +90° = pointing down (toward bottom wall)
• Left turns = negative angles
• Right turns = positive angles

Arc System:
• Radius positive, Angle positive = Forward and to the right
• Radius positive, Angle negative = Forward and to the left
• Radius negative, Angle positive = Backward and to the right  
• Radius negative, Angle negative = Backward and to the left
        """
        
        instruction_text = tk.Text(instructions_frame, height=8, width=35, wrap=tk.WORD)
        instruction_text.pack(fill=tk.BOTH, expand=True)
        instruction_text.insert(tk.END, instructions)
        instruction_text.config(state=tk.DISABLED)
        
    def setup_drive_params(self):
        """Setup parameters for drive mode."""
        # Clear existing widgets
        for widget in self.params_frame.winfo_children():
            widget.destroy()
        
        if self.mode_var.get() == "drive":
            # Speed
            ttk.Label(self.params_frame, text="Speed (mm/s):").pack(anchor=tk.W)
            self.speed_var = tk.StringVar(value="400")
            ttk.Entry(self.params_frame, textvariable=self.speed_var).pack(fill=tk.X, pady=(0, 10))
            
            # Backward checkbox
            self.backward_var = tk.BooleanVar()
            ttk.Checkbutton(self.params_frame, text="Backward", variable=self.backward_var).pack(anchor=tk.W, pady=(0, 10))
            
            # Target angle
            ttk.Label(self.params_frame, text="Target Angle (deg):").pack(anchor=tk.W)
            self.target_angle_var = tk.StringVar(value="0")
            ttk.Entry(self.params_frame, textvariable=self.target_angle_var).pack(fill=tk.X, pady=(0, 10))
            
            # Line detection
            self.till_black_var = tk.BooleanVar()
            ttk.Checkbutton(self.params_frame, text="Stop at Black Line", variable=self.till_black_var).pack(anchor=tk.W)
            
            self.till_white_var = tk.BooleanVar()
            ttk.Checkbutton(self.params_frame, text="Stop at White Line", variable=self.till_white_var).pack(anchor=tk.W)
            
        elif self.mode_var.get() == "turn":
            # Speed
            ttk.Label(self.params_frame, text="Speed (mm/s):").pack(anchor=tk.W)
            self.speed_var = tk.StringVar(value="400")
            ttk.Entry(self.params_frame, textvariable=self.speed_var).pack(fill=tk.X, pady=(0, 10))
            
            # Force turn
            self.force_turn_var = tk.StringVar(value="none")
            ttk.Label(self.params_frame, text="Force Turn:").pack(anchor=tk.W)
            ttk.Radiobutton(self.params_frame, text="None", variable=self.force_turn_var, value="none").pack(anchor=tk.W)
            ttk.Radiobutton(self.params_frame, text="Left", variable=self.force_turn_var, value="left").pack(anchor=tk.W)
            ttk.Radiobutton(self.params_frame, text="Right", variable=self.force_turn_var, value="right").pack(anchor=tk.W)
            
        elif self.mode_var.get() == "curve":
            # Radius
            ttk.Label(self.params_frame, text="Radius (mm):").pack(anchor=tk.W)
            self.radius_var = tk.StringVar(value="100")
            radius_entry = ttk.Entry(self.params_frame, textvariable=self.radius_var)
            radius_entry.pack(fill=tk.X, pady=(0, 10))
            radius_entry.bind('<KeyRelease>', self.on_curve_param_change)
            
            # Speed
            ttk.Label(self.params_frame, text="Speed (mm/s):").pack(anchor=tk.W)
            self.speed_var = tk.StringVar(value="300")
            ttk.Entry(self.params_frame, textvariable=self.speed_var).pack(fill=tk.X, pady=(0, 10))
            
            # Arc type selection
            self.arc_type_var = tk.StringVar(value="angle")
            ttk.Label(self.params_frame, text="Arc Type:").pack(anchor=tk.W)
            ttk.Radiobutton(self.params_frame, text="By Angle", variable=self.arc_type_var, value="angle", 
                           command=self.on_arc_type_change).pack(anchor=tk.W)
            ttk.Radiobutton(self.params_frame, text="By Distance", variable=self.arc_type_var, value="distance", 
                           command=self.on_arc_type_change).pack(anchor=tk.W)
            
            # Angle/Distance parameter
            self.arc_param_var = tk.StringVar(value="90")
            self.arc_param_label = ttk.Label(self.params_frame, text="Angle (degrees):")
            self.arc_param_label.pack(anchor=tk.W)
            arc_param_entry = ttk.Entry(self.params_frame, textvariable=self.arc_param_var)
            arc_param_entry.pack(fill=tk.X, pady=(0, 10))
            arc_param_entry.bind('<KeyRelease>', self.on_curve_param_change)
            
            # Then parameter
            ttk.Label(self.params_frame, text="Then:").pack(anchor=tk.W)
            self.then_var = tk.StringVar(value="HOLD")
            then_combo = ttk.Combobox(self.params_frame, textvariable=self.then_var, 
                                     values=["HOLD", "BRAKE", "COAST"])
            then_combo.pack(fill=tk.X, pady=(0, 10))
            
            # Wait checkbox
            self.wait_var = tk.BooleanVar(value=True)
            ttk.Checkbutton(self.params_frame, text="Wait for completion", 
                           variable=self.wait_var).pack(anchor=tk.W)
            
        elif self.mode_var.get() == "line_follow":
            # Speed
            ttk.Label(self.params_frame, text="Speed (mm/s):").pack(anchor=tk.W)
            self.speed_var = tk.StringVar(value="200")
            ttk.Entry(self.params_frame, textvariable=self.speed_var).pack(fill=tk.X, pady=(0, 10))
            
            # Edge
            self.edge_var = tk.StringVar(value="left")
            ttk.Label(self.params_frame, text="Edge:").pack(anchor=tk.W)
            ttk.Radiobutton(self.params_frame, text="Left", variable=self.edge_var, value="left").pack(anchor=tk.W)
            ttk.Radiobutton(self.params_frame, text="Right", variable=self.edge_var, value="right").pack(anchor=tk.W)
            
            # Control color
            ttk.Label(self.params_frame, text="Control Color:").pack(anchor=tk.W)
            self.control_color_var = tk.StringVar(value="63")
            ttk.Entry(self.params_frame, textvariable=self.control_color_var).pack(fill=tk.X, pady=(0, 10))
            
        elif self.mode_var.get() == "pickers":
            # Position
            positions = ["ALL_UP", "BELOW_LID", "HALF_LIFT_LID", "LIFT_LID", "DROP_BLOCKS", 
                        "DROP_BLOCKS_LOWER", "PICK_RED", "PICK_WHITE", "PICK_GREEN", 
                        "PICK_YELLOW", "HOLD_DRONE", "GROUND", "MIDDLE_DROP_BLOCKS"]
            ttk.Label(self.params_frame, text="Position:").pack(anchor=tk.W)
            self.picker_position_var = tk.StringVar(value="ALL_UP")
            picker_combo = ttk.Combobox(self.params_frame, textvariable=self.picker_position_var, values=positions)
            picker_combo.pack(fill=tk.X, pady=(0, 10))
            
            # Speed
            ttk.Label(self.params_frame, text="Speed:").pack(anchor=tk.W)
            self.speed_var = tk.StringVar(value="700")
            ttk.Entry(self.params_frame, textvariable=self.speed_var).pack(fill=tk.X, pady=(0, 10))
            
        elif self.mode_var.get() == "ball_picker":
            # Position
            positions = ["PICK_BALLS", "DROP_BALLS", "SLIDE_BALL", "ROVER"]
            ttk.Label(self.params_frame, text="Position:").pack(anchor=tk.W)
            self.ball_picker_position_var = tk.StringVar(value="PICK_BALLS")
            ball_combo = ttk.Combobox(self.params_frame, textvariable=self.ball_picker_position_var, values=positions)
            ball_combo.pack(fill=tk.X, pady=(0, 10))
            
            # Speed
            ttk.Label(self.params_frame, text="Speed:").pack(anchor=tk.W)
            self.speed_var = tk.StringVar(value="1000")
            ttk.Entry(self.params_frame, textvariable=self.speed_var).pack(fill=tk.X, pady=(0, 10))
            
        elif self.mode_var.get() == "wait":
            # Wait time
            ttk.Label(self.params_frame, text="Wait Time (ms):").pack(anchor=tk.W)
            self.wait_time_var = tk.StringVar(value="1000")
            ttk.Entry(self.params_frame, textvariable=self.wait_time_var).pack(fill=tk.X, pady=(0, 10))
    
    def setup_plot_area(self, parent):
        """Setup the matplotlib plot area."""
        # Create figure
        self.fig = Figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111)
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add toolbar
        toolbar = NavigationToolbar2Tk(self.canvas, parent)
        toolbar.update()
        
        # Connect events
        self.canvas.mpl_connect('button_press_event', self.on_mouse_press)
        self.canvas.mpl_connect('button_release_event', self.on_mouse_release)
        self.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        
        # Connect keyboard events
        self.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Bind keyboard events to the root window
        self.root.bind('<Left>', self.on_left_arrow)
        self.root.bind('<Right>', self.on_right_arrow)
        self.root.bind('<Up>', self.on_up_arrow)
        self.root.bind('<Down>', self.on_down_arrow)
        self.root.focus_set()  # Set focus to capture keyboard events
        
        self.update_plot()
        
    def update_plot(self):
        """Update the plot with current path."""
        if self.ax is None:
            return
        self.ax.clear()
        
        # Set up plot
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_title('Interactive Path Drawing - WRO 2025 Junior Field')
        
        # Display field image if available (optimized for performance)
        if self.field_image is not None:
            # Display the field image as background with reduced alpha for better performance
            self.ax.imshow(self.field_image, extent=(0, self.planner.field_width, 0, self.planner.field_height), alpha=0.3)
        
        # Draw field boundaries
        field_rect = patches.Rectangle((0, 0), self.planner.field_width, self.planner.field_height, 
                                     linewidth=2, edgecolor='black', facecolor='none')
        self.ax.add_patch(field_rect)
        
        # Draw grid
        for x in range(0, int(self.planner.field_width) + 1, 10):
            self.ax.axvline(x=x, color='gray', alpha=0.3, linewidth=0.5)
        for y in range(0, int(self.planner.field_height) + 1, 10):
            self.ax.axhline(y=y, color='gray', alpha=0.3, linewidth=0.5)
        
        # Draw path history
        if len(self.planner.path_history) > 1:
            x_coords = [pos[0] for pos in self.planner.path_history]
            y_coords = [pos[1] for pos in self.planner.path_history]
            self.ax.plot(x_coords, y_coords, 'b-', linewidth=2, alpha=0.7, label='Robot Path')
            
            # Draw arrows
            for i in range(0, len(self.planner.path_history) - 1, max(1, len(self.planner.path_history) // 20)):
                x, y, heading = self.planner.path_history[i]
                dx = math.cos(math.radians(heading)) * 5
                dy = math.sin(math.radians(heading)) * 5
                self.ax.arrow(x, y, dx, dy, head_width=2, head_length=2, fc='blue', ec='blue', alpha=0.7)
        
        # Draw current drawing
        if self.current_path_points:
            if self.mode_var.get() == "curve":
                # Draw arc preview from robot's current position
                try:
                    radius = float(self.radius_var.get())
                    if radius != 0:
                        # Draw arc preview using the radius
                        self._draw_arc_preview(None, None, radius)
                    else:
                        # Fallback to straight line if radius is 0
                        x_coords = [p[0] for p in self.current_path_points]
                        y_coords = [p[1] for p in self.current_path_points]
                        self.ax.plot(x_coords, y_coords, 'r--', linewidth=2, alpha=0.8)
                except ValueError:
                    # Fallback to straight line if radius is invalid
                    x_coords = [p[0] for p in self.current_path_points]
                    y_coords = [p[1] for p in self.current_path_points]
                    self.ax.plot(x_coords, y_coords, 'r--', linewidth=2, alpha=0.8)
            else:
                x_coords = [p[0] for p in self.current_path_points]
                y_coords = [p[1] for p in self.current_path_points]
                self.ax.plot(x_coords, y_coords, 'r--', linewidth=2, alpha=0.8)
        
        # Draw robot visualization
        if self.show_robot:
            # Draw current robot position (larger and more prominent)
            # Create rectangle centered at robot position
            robot_rect = patches.Rectangle(
                (-self.robot_size/2, -self.robot_size/2),  # Position relative to center
                self.robot_size, self.robot_size,
                linewidth=2, edgecolor='blue', facecolor='blue', alpha=0.6
            )
            
            # Apply rotation and translation to center the robot
            transform = (Affine2D().rotate_deg(self.planner.heading) + 
                        Affine2D().translate(self.planner.x, self.planner.y))
            robot_rect.set_transform(transform + self.ax.transData)
            self.ax.add_patch(robot_rect)
            
            # Draw heading indicator
            dx = math.cos(math.radians(self.planner.heading)) * (self.robot_size/2 + 3)
            dy = math.sin(math.radians(self.planner.heading)) * (self.robot_size/2 + 3)
            self.ax.arrow(self.planner.x, self.planner.y, dx, dy, head_width=2, head_length=2, fc='white', ec='white', linewidth=2)
            
            # Draw historical robot positions (smaller and faded)
            for i, (x, y, heading) in enumerate(self.planner.path_history):
                if i % max(1, len(self.planner.path_history) // 10) == 0:  # Show every 10th position
                    # Create rectangle centered at historical position
                    robot_rect = patches.Rectangle(
                        (-self.robot_size/2, -self.robot_size/2),  # Position relative to center
                        self.robot_size, self.robot_size,
                        linewidth=1, edgecolor='red', facecolor='red', alpha=0.2
                    )
                    
                    # Apply rotation and translation to center the robot
                    transform = (Affine2D().rotate_deg(heading) + 
                                Affine2D().translate(x, y))
                    robot_rect.set_transform(transform + self.ax.transData)
                    self.ax.add_patch(robot_rect)
                    
                    # Draw heading indicator
                    dx = math.cos(math.radians(heading)) * (self.robot_size/2 + 2)
                    dy = math.sin(math.radians(heading)) * (self.robot_size/2 + 2)
                    self.ax.arrow(x, y, dx, dy, head_width=1, head_length=1, fc='red', ec='red', alpha=0.5)
        
        # Draw start point
        if self.start_point:
            self.ax.plot(self.start_point[0], self.start_point[1], 'go', markersize=10, label='Start Point')
        
        # Set limits
        margin = 20
        self.ax.set_xlim(-margin, self.planner.field_width + margin)
        self.ax.set_ylim(-margin, self.planner.field_height + margin)
        
        # Add legend only if there are labeled artists
        if self.ax.get_legend_handles_labels()[0]:
            self.ax.legend()
        
        # Update info
        self.update_info()
        
        if self.canvas is not None:
            self.canvas.draw()
        
    def update_info(self):
        """Update the information display."""
        summary = self.planner.get_path_summary()
        
        info_text = f"Total Distance: {summary['total_distance_cm']:.1f} cm\n"
        info_text += f"Estimated Time: {summary['estimated_time_seconds']:.1f} s\n"
        info_text += f"Commands: {summary['number_of_commands']}\n"
        info_text += f"Current Position: ({self.planner.x:.1f}, {self.planner.y:.1f}, {self.planner.heading:.1f}°)\n\n"
        
        info_text += "Commands:\n"
        for i, cmd in enumerate(self.planner.commands):
            info_text += f"{i+1}. {cmd['type']}\n"
        
        self.info_text.delete(1.0, tk.END)
        self.info_text.insert(tk.END, info_text)
        
    def on_mode_change(self):
        """Handle mode change."""
        self.setup_drive_params()
        # If switching to curve mode, show preview immediately
        if self.mode_var.get() == "curve" and self.ax is not None:
            self.current_path_points = [(0, 0)]  # Dummy point to trigger preview
            self.update_plot()
        
    def on_arc_type_change(self):
        """Handle arc type change (angle vs distance)."""
        if hasattr(self, 'arc_param_label'):
            if self.arc_type_var.get() == "angle":
                self.arc_param_label.config(text="Angle (degrees):")
                if not self.arc_param_var.get() or self.arc_param_var.get().isdigit():
                    self.arc_param_var.set("90")
            else:  # distance
                self.arc_param_label.config(text="Distance (mm):")
                if not self.arc_param_var.get() or self.arc_param_var.get().isdigit():
                    self.arc_param_var.set("157")  # ~90 degrees for radius 100
        
        # Update curve preview if in curve mode
        if self.mode_var.get() == "curve":
            self.update_plot()
    
    def on_curve_param_change(self, event):
        """Handle curve parameter changes to update preview."""
        if self.mode_var.get() == "curve":
            # Use after() to debounce rapid updates
            if hasattr(self, '_update_timer'):
                self.root.after_cancel(self._update_timer)
            self._update_timer = self.root.after(100, self.update_plot)
                    
    def update_direction_indicator(self):
        """Update the direction indicator based on current turn angle."""
        angle = self.turn_angle
        if angle == 0:
            direction = "→ Right"
        elif angle == 90:
            direction = "↓ Down"
        elif angle == -90:
            direction = "↑ Up"
        elif angle == 180 or angle == -180:
            direction = "← Left"
        elif -90 < angle < 0:
            direction = "↗ Up-Right"
        elif 0 < angle < 90:
            direction = "↘ Down-Right"
        elif 90 < angle < 180:
            direction = "↙ Down-Left"
        elif -180 < angle < -90:
            direction = "↖ Up-Left"
        else:
            direction = f"{angle}°"
        
        self.direction_var.set(direction)
        
    def on_continue_change(self):
        """Handle continue path checkbox change."""
        self.continue_path = self.continue_var.get()
        
    def on_start_pos_change(self, event=None):
        """Handle start position change."""
        # Move robot to the new start position immediately
        x, y, heading = self.get_start_position()
        
        # Clear the current path and move robot to new position
        self.planner.commands = []
        self.planner.x = x
        self.planner.y = y
        self.planner.heading = heading
        self.planner.path_history = [(x, y, heading)]
        
        # Update the display
        self.update_plot()
        
    def get_start_position(self):
        """Get the current start position coordinates."""
        if self.start_pos_var.get() == "bottom_left":
            # Bottom left (RobotMission box)
            return 20, 20, 0
        else:
            # Center far side (center Y-axis, far side X-axis)
            return self.planner.field_width - 20, self.planner.field_height / 2, 180
        
    def reset_to_origin(self):
        """Reset robot position to selected start position."""
        x, y, heading = self.get_start_position()
        self.planner.reset_position(x, y, heading)
        self.update_plot()
        
    def on_key_press(self, event):
        """Handle matplotlib key press events."""
        if event.key == 'left':
            self.on_left_arrow(None)
        elif event.key == 'right':
            self.on_right_arrow(None)
        elif event.key == 'up':
            self.on_up_arrow(None)
        elif event.key == 'down':
            self.on_down_arrow(None)
            
    def on_left_arrow(self, event):
        """Handle left arrow key press."""
        if self.robot_preview_mode:
            # Turn robot left (negative angle)
            self.planner.heading -= 5
        else:
            # Adjust turn angle - left arrow decreases angle (toward -90°)
            self.turn_angle -= self.angle_step
            self.angle_var.set(f"{self.turn_angle}°")
            self.update_direction_indicator()
        self.update_plot()
        
    def on_right_arrow(self, event):
        """Handle right arrow key press."""
        if self.robot_preview_mode:
            # Turn robot right (positive angle)
            self.planner.heading += 5
        else:
            # Adjust turn angle - right arrow increases angle (toward +90°)
            self.turn_angle += self.angle_step
            self.angle_var.set(f"{self.turn_angle}°")
            self.update_direction_indicator()
        self.update_plot()
        
    def on_up_arrow(self, event):
        """Handle up arrow key press."""
        if self.robot_preview_mode:
            # Move robot forward
            self.move_robot_preview(0, 5)
        else:
            # Adjust turn angle
            self.turn_angle += self.angle_step * 3  # 15 degrees
            self.angle_var.set(f"{self.turn_angle}°")
            self.update_direction_indicator()
        
    def on_down_arrow(self, event):
        """Handle down arrow key press."""
        if self.robot_preview_mode:
            # Move robot backward
            self.move_robot_preview(0, -5)
        else:
            # Adjust turn angle
            self.turn_angle -= self.angle_step * 3  # 15 degrees
            self.angle_var.set(f"{self.turn_angle}°")
            self.update_direction_indicator()
        
    def on_robot_size_change(self, event):
        """Handle robot size change."""
        try:
            self.robot_size = float(self.robot_size_var.get())
            # Use after() to debounce rapid updates
            if hasattr(self, '_robot_size_timer'):
                self.root.after_cancel(self._robot_size_timer)
            self._robot_size_timer = self.root.after(200, self.update_plot)
        except ValueError:
            pass  # Invalid input, ignore
            
    def on_show_robot_change(self):
        """Handle show robot checkbox change."""
        self.show_robot = self.show_robot_var.get()
        self.update_plot()
        
    def on_preview_mode_change(self):
        """Handle preview mode checkbox change."""
        self.robot_preview_mode = self.preview_mode_var.get()
        if self.robot_preview_mode:
            # Update instructions for preview mode
            self.angle_var.set("Preview Mode Active")
            self.direction_var.set("Move with Arrow Keys")
        else:
            # Reset to turn angle display
            self.angle_var.set(f"{self.turn_angle}°")
            self.update_direction_indicator()
            
    def move_robot_preview(self, dx, dy):
        """Move robot in preview mode."""
        if not self.robot_preview_mode:
            return
            
        # Calculate new position
        new_x = self.planner.x + dx
        new_y = self.planner.y + dy
        
        # Keep robot within field bounds
        new_x = max(0, min(self.planner.field_width, new_x))
        new_y = max(0, min(self.planner.field_height, new_y))
        
        # Update robot position (without adding to path history)
        self.planner.x = new_x
        self.planner.y = new_y
        
        # Update display
        self.update_plot()
        
    def on_mouse_press(self, event):
        """Handle mouse press event."""
        if event.inaxes != self.ax:
            return
        
        if event.button == 1:  # Left click
            self.is_drawing = True
            if self.mode_var.get() == "curve":
                # For arc: just add the arc command based on current parameters
                self.add_arc_segment()
                self.is_drawing = False
                self.start_point = None
                self.current_path_points = []
                self.update_plot()
                return
            else:
                self.start_point = (event.xdata, event.ydata)
                self.current_path_points = [self.start_point]
            # For turn mode, immediately add the turn command
            if self.mode_var.get() == "turn":
                self.add_turn_command()
                self.is_drawing = False
                self.start_point = None
                self.current_path_points = []
                self.update_plot()
            
    def on_mouse_move(self, event):
        """Handle mouse move event."""
        if not self.is_drawing or event.inaxes != self.ax:
            return
        if self.mode_var.get() == "curve":
            # For curve mode, just show the preview based on current parameters
            # No need to track mouse movement for curves
            return
        if self.start_point:
            self.current_path_points.append((event.xdata, event.ydata))
            self.update_plot()
            
    def on_mouse_release(self, event):
        """Handle mouse release event."""
        if not self.is_drawing or event.inaxes != self.ax:
            return
        if self.mode_var.get() == "curve":
            # Handled in on_mouse_press for arc
            return
        if event.button == 1 and self.start_point:  # Left click release
            end_point = (event.xdata, event.ydata)
            self.add_path_segment(self.start_point, end_point)
        self.is_drawing = False
        self.start_point = None
        self.current_path_points = []
        self.update_plot()
        
    def add_turn_command(self):
        """Add a turn command using the current turn angle."""
        try:
            speed = int(self.speed_var.get())
            force_turn = self.force_turn_var.get()
            
            self.planner.turn_to_angle(
                target_angle=-self.turn_angle,
                speed=speed,
                force_turn=force_turn if force_turn != "none" else None
            )
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid parameter value: {e}")
        
    def add_path_segment(self, start, end, control=None):
        """Add a path segment based on current mode. For curve mode, control is the Bezier control point."""
        mode = self.mode_var.get()
        try:
            if mode == "drive":
                # Calculate distance and angle
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                distance = math.sqrt(dx*dx + dy*dy)
                drive_angle = math.degrees(math.atan2(dy, dx))  # Angle calculation
                
                # Get parameters
                speed = int(self.speed_var.get())
                backward = self.backward_var.get()
                target_angle = float(self.target_angle_var.get()) if self.target_angle_var.get() else drive_angle
                till_black = self.till_black_var.get()
                till_white = self.till_white_var.get()
                
                # If continuing path, use current robot heading as target angle
                if self.continue_path and not target_angle:
                    target_angle = self.planner.heading
                
                self.planner.drive_straight(
                    distance=distance,
                    speed=speed,
                    backward=backward,
                    target_angle=target_angle,
                    till_black_line=till_black,
                    till_white_line=till_white
                )
                
            elif mode == "turn":
                # Turn angle is controlled by keyboard, not mouse drag
                # This should not be reached due to immediate execution in mouse_press
                pass
                
            elif mode == "curve":
                # Arc: use the add_arc_segment method instead
                # This should not be reached as curves are handled in on_mouse_press
                pass
                
            elif mode == "line_follow":
                # Calculate distance
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                distance = math.sqrt(dx*dx + dy*dy) * 10  # Convert to mm
                
                speed = int(self.speed_var.get())
                edge = self.edge_var.get()
                control_color = int(self.control_color_var.get())
                
                self.planner.follow_line(
                    distance=distance,
                    speed=speed,
                    edge=edge,
                    control_color=control_color
                )
                
            elif mode == "pickers":
                position = self.picker_position_var.get()
                speed = int(self.speed_var.get())
                
                self.planner.set_pickers_position(
                    position=position,
                    speed=speed
                )
                
            elif mode == "ball_picker":
                position = self.ball_picker_position_var.get()
                speed = int(self.speed_var.get())
                
                self.planner.set_ball_picker_position(
                    position=position,
                    speed=speed
                )
                
            elif mode == "wait":
                wait_time = int(self.wait_time_var.get())
                self.planner.wait(wait_time)
                
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid parameter value: {e}")
            
    def add_arc_segment(self, start=None, end=None):
        """Add an arc segment based on robot's current position and parameters."""
        try:
            # Get parameters from UI
            radius = float(self.radius_var.get())
            speed = int(self.speed_var.get())
            arc_type = self.arc_type_var.get()
            arc_param = float(self.arc_param_var.get())
            then = self.then_var.get()
            wait = self.wait_var.get()
            
            # Determine if we should use angle or distance
            if arc_type == "angle":
                angle = arc_param
                distance = None
            else:  # distance
                angle = None
                distance = arc_param
            
            # Determine radius sign based on the awaitarc specification:
            # Positive radius = right turn, Negative radius = left turn
            # We'll use the sign from the UI radius directly
            # (The UI allows negative radius for left turns, positive for right turns)
            
            # Call the awaitarc method
            self.planner.awaitarc(
                radius=radius,
                angle=angle,
                distance=distance,
                speed=speed,
                then=then,
                wait=wait
            )
            
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid parameter value: {e}")
            
    def clear_path(self):
        """Clear the entire path."""
        x, y, heading = self.get_start_position()
        self.planner.reset_position(x, y, heading)
        self.update_plot()
        
    def undo_last(self):
        """Undo the last command."""
        if self.planner.commands:
            # Remove the last command
            self.planner.commands.pop()
            
            # Save the remaining commands
            remaining_commands = self.planner.commands.copy()
            
            # Clear all commands and reset position
            self.planner.commands = []
            x, y, heading = self.get_start_position()
            self.planner.x = x
            self.planner.y = y
            self.planner.heading = heading
            self.planner.path_history = [(x, y, heading)]
            
            # Replay all remaining commands to rebuild the path
            for cmd in remaining_commands:
                self.replay_command(cmd)
            
            # Update the display
            self.update_plot()
            
    def replay_command(self, cmd):
        """Replay a command to rebuild path."""
        if cmd['type'] == 'drive_straight':
            # Extract parameters, excluding 'type'
            params = {k: v for k, v in cmd.items() if k != 'type'}
            # Update position without adding to commands list
            self._replay_drive_straight(**params)
        elif cmd['type'] == 'turn_to_angle':
            # Extract parameters, excluding 'type'
            params = {k: v for k, v in cmd.items() if k != 'type'}
            # The stored target_angle is negated, so we need to negate it back
            if 'target_angle' in params:
                params['target_angle'] = -params['target_angle']
            # Update position without adding to commands list
            self._replay_turn_to_angle(**params)
        elif cmd['type'] == 'curve':
            params = {k: v for k, v in cmd.items() if k != 'type'}
            # Update position without adding to commands list
            self._replay_curve(**params)
        elif cmd['type'] == 'awaitarc':
            params = {k: v for k, v in cmd.items() if k != 'type'}
            # Update position without adding to commands list
            self._replay_awaitarc(**params)
        elif cmd['type'] == 'bezier_curve':
            params = {k: v for k, v in cmd.items() if k != 'type'}
            # Update position without adding to commands list
            self._replay_bezier_curve(**params)
        elif cmd['type'] == 'follow_line':
            params = {k: v for k, v in cmd.items() if k != 'type'}
            # Update position without adding to commands list
            self._replay_follow_line(**params)
        elif cmd['type'] == 'set_pickers_position':
            params = {k: v for k, v in cmd.items() if k != 'type'}
            # Update position without adding to commands list
            self._replay_set_pickers_position(**params)
        elif cmd['type'] == 'set_ball_picker_position':
            params = {k: v for k, v in cmd.items() if k != 'type'}
            # Update position without adding to commands list
            self._replay_set_ball_picker_position(**params)
        elif cmd['type'] == 'wait':
            params = {k: v for k, v in cmd.items() if k != 'type'}
            # Update position without adding to commands list
            self._replay_wait(**params)
    
    def _replay_drive_straight(self, distance, speed=400, backward=False, target_angle=None, 
                              till_black_line=False, till_white_line=False, detect_stall=False,
                              stop_when_load_above=0, slow_down=True, slow_speed_override=50):
        """Replay drive_straight without adding to commands list."""
        self.planner._update_position_after_drive(distance, backward)
        # Add the command back to the list
        cmd = {
            'type': 'drive_straight',
            'distance': distance,
            'speed': speed,
            'backward': backward,
            'target_angle': target_angle,
            'till_black_line': till_black_line,
            'till_white_line': till_white_line,
            'detect_stall': detect_stall,
            'stop_when_load_above': stop_when_load_above,
            'slow_down': slow_down,
            'slow_speed_override': slow_speed_override
        }
        self.planner.commands.append(cmd)
    
    def _replay_turn_to_angle(self, target_angle, speed=400, force_turn=None, one_wheel_turn=False):
        """Replay turn_to_angle without adding to commands list."""
        # Calculate angle change
        current_angle = self.planner.heading
        angle_diff = target_angle - current_angle
        
        # Find shortest path
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
            
        # Update position
        self.planner._update_position_after_turn(angle_diff)
        
        # Add the command back to the list
        cmd = {
            'type': 'turn_to_angle',
            'target_angle': -target_angle,
            'speed': speed,
            'force_turn': force_turn,
            'one_wheel_turn': one_wheel_turn
        }
        self.planner.commands.append(cmd)
    
    def _replay_curve(self, radius, angle, speed=300, acceleration=450, deceleration=0, 
                     dont_accelerate=False, dont_decelerate=False):
        """Replay curve without adding to commands list."""
        self.planner._update_position_after_curve(radius, angle)
        
        # Add the command back to the list
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
        self.planner.commands.append(cmd)
    
    def _replay_awaitarc(self, radius, angle=None, distance=None, speed=300, then="HOLD", wait=True):
        """Replay awaitarc without adding to commands list."""
        # Calculate the angle if distance is provided
        if distance is not None:
            angle = math.degrees(distance / radius)
        
        self.planner._update_position_after_curve(radius, angle)
        
        # Add the command back to the list
        cmd = {
            'type': 'awaitarc',
            'radius': radius,
            'angle': angle,
            'distance': distance,
            'speed': speed,
            'then': then,
            'wait': wait
        }
        self.planner.commands.append(cmd)
    
    def _replay_bezier_curve(self, start, control, end, speed=300, acceleration=450):
        """Replay bezier_curve without adding to commands list."""
        points = self.planner._bezier_curve_points(start, control, end, num=50)
        for i, (x, y) in enumerate(points[1:], 1):
            self.planner.x = x
            self.planner.y = y
            t = i / (len(points) - 1)
            # Calculate tangent vector (derivative of Bezier curve)
            dx = 2*(1-t)*(control[0]-start[0]) + 2*t*(end[0]-control[0])
            dy = 2*(1-t)*(control[1]-start[1]) + 2*t*(end[1]-control[1])
            if dx != 0 or dy != 0:
                # Use positive atan2 for correct direction
                self.planner.heading = math.degrees(math.atan2(dy, dx))
            self.planner._add_to_history()
        
        # Add the command back to the list
        cmd = {
            'type': 'bezier_curve',
            'start': start,
            'control': control,
            'end': end,
            'speed': speed,
            'acceleration': acceleration
        }
        self.planner.commands.append(cmd)
    
    def _replay_follow_line(self, distance, speed, edge='left', control_color=63, 
                           kp=0.5, ki=0, kd=0, slow_start=True, slow_down=True,
                           slow_speed_override=100, start_cm_slow=0):
        """Replay follow_line without adding to commands list."""
        # For line following, we'll approximate as a straight line
        distance_cm = distance / 10
        self.planner._update_position_after_drive(distance_cm)
        
        # Add the command back to the list
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
        self.planner.commands.append(cmd)
    
    def _replay_set_pickers_position(self, position, wait=True, speed=700):
        """Replay set_pickers_position without adding to commands list."""
        # This doesn't change robot position, just add to commands
        cmd = {
            'type': 'set_pickers_position',
            'position': position,
            'wait': wait,
            'speed': speed
        }
        self.planner.commands.append(cmd)
    
    def _replay_set_ball_picker_position(self, position, wait=True, speed=1000):
        """Replay set_ball_picker_position without adding to commands list."""
        # This doesn't change robot position, just add to commands
        cmd = {
            'type': 'set_ball_picker_position',
            'position': position,
            'wait': wait,
            'speed': speed
        }
        self.planner.commands.append(cmd)
    
    def _replay_wait(self, milliseconds):
        """Replay wait without adding to commands list."""
        cmd = {
            'type': 'wait',
            'milliseconds': milliseconds
        }
        self.planner.commands.append(cmd)
            
    def generate_code(self):
        """Generate and display code."""
        if not self.planner.commands:
            messagebox.showwarning("Warning", "No path to generate code from!")
            return
            
        # Generate base code
        code = self.planner.generate_code("drawn_path")
        
        # Add waitForRightButtonPress() after drive and turn commands if checkbox is checked
        if self.wait_button_var.get():
            lines = code.split('\n')
            modified_lines = []
            
            for line in lines:
                modified_lines.append(line)
                # Check if this is a drive or turn command
                if any(cmd in line for cmd in ['gyroStraightWithDrive(', 'turnToAngle(', 'curve(', 'awaitarc(', 'followBlackLinePID(']):
                    # Add wait command after the command
                    modified_lines.append("    waitForRightButtonPress()")
            
            code = '\n'.join(modified_lines)
        
        # Create code window
        code_window = tk.Toplevel(self.root)
        code_window.title("Generated Code")
        code_window.geometry("800x600")
        
        # Add text widget
        text_widget = tk.Text(code_window, wrap=tk.WORD, font=("Courier", 10))
        text_widget.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Add scrollbar
        scrollbar = ttk.Scrollbar(code_window, orient=tk.VERTICAL, command=text_widget.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        text_widget.configure(yscrollcommand=scrollbar.set)
        
        # Insert code
        text_widget.insert(tk.END, code)
        text_widget.config(state=tk.DISABLED)
        
        # Add copy button
        def copy_code():
            code_window.clipboard_clear()
            code_window.clipboard_append(code)
            messagebox.showinfo("Success", "Code copied to clipboard!")
        ttk.Button(code_window, text="Copy to Clipboard", command=copy_code).pack(pady=10)

        # Add Run in Simulator button
        def run_in_simulator():
            import tempfile
            import importlib.util
            import traceback
            import os
            from tkinter import messagebox
            try:
                # Write code to a temporary file
                with tempfile.NamedTemporaryFile('w', suffix='.py', delete=False) as tmp:
                    tmp.write(code)
                    tmp_path = tmp.name
                # Dynamically import the generated code
                spec = importlib.util.spec_from_file_location("drawn_path_module", tmp_path)
                if spec is None or spec.loader is None:
                    raise ImportError("Could not load generated code for simulation.")
                drawn_path_module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(drawn_path_module)
                # Import simulator_utils and inject its symbols
                import simulator_utils
                for name in dir(simulator_utils):
                    if not name.startswith("__"):
                        setattr(drawn_path_module, name, getattr(simulator_utils, name))
                # Initialize simulator graphics if available
                if hasattr(simulator_utils, 'initializeGraphics'):
                    simulator_utils.initializeGraphics()
                # Run the generated path function in the simulator
                if hasattr(drawn_path_module, 'drawn_path'):
                    drawn_path_module.drawn_path()
                else:
                    raise Exception("Generated code does not contain a 'drawn_path' function.")
                # Optionally, call simulator_utils.runSimulator() if needed
                if hasattr(simulator_utils, 'runSimulator'):
                    simulator_utils.runSimulator()
            except Exception as e:
                tb = traceback.format_exc()
                messagebox.showerror("Simulator Error", f"An error occurred while running the code in the simulator:\n{e}\n\nTraceback:\n{tb}")
            finally:
                try:
                    os.remove(tmp_path)
                except Exception:
                    pass
        ttk.Button(code_window, text="Run in Simulator", command=run_in_simulator).pack(pady=10)
        
    def save_path(self):
        """Save the current path."""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            self.planner.save_path(filename)
            messagebox.showinfo("Success", f"Path saved to {filename}")
            
    def load_path(self):
        """Load a saved path."""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            try:
                self.planner.load_path(filename)
                self.update_plot()
                messagebox.showinfo("Success", f"Path loaded from {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load path: {e}")

    def _bezier_curve_points(self, p0, p1, p2, num=50):
        """Return a list of points along a quadratic Bezier curve."""
        points = []
        for t in np.linspace(0, 1, num):
            x = (1-t)**2 * p0[0] + 2*(1-t)*t*p1[0] + t**2*p2[0]
            y = (1-t)**2 * p0[1] + 2*(1-t)*t*p1[1] + t**2*p2[1]
            points.append((x, y))
        return points
        
    def _draw_arc_preview(self, start, end, radius):
        """Draw an arc preview from robot's current position with given radius and angle."""
        if self.ax is None:
            return
        try:
            # Convert radius to cm for display
            radius_cm = abs(radius) / 10
            
            # For robot movement, the arc should start from the robot's current position
            # and follow the robot's current heading
            robot_x = self.planner.x
            robot_y = self.planner.y
            robot_heading = self.planner.heading
            
            # Calculate the center of the circle
            # Match the path_planner.py logic exactly
            heading_rad = math.radians(robot_heading)
            radius_cm = radius / 10.0
            
            # WRO coordinate system: 0° = right (+X), 90° = down (+Y), 180° = left (-X), 270° = up (-Y)
            # Front/back is determined by sign of radius, left/right is determined by sign of angle
            # Positive radius = forward, negative radius = backward
            # Positive angle = right turn, negative angle = left turn
            
            # Calculate center of curvature (matching path_planner.py)
            if radius > 0:  # Forward
                center_x = robot_x - radius_cm * math.sin(heading_rad)
                center_y = robot_y + radius_cm * math.cos(heading_rad)
            else:  # Backward
                center_x = robot_x + radius_cm * math.sin(heading_rad)
                center_y = robot_y - radius_cm * math.cos(heading_rad)
            
            # Get the angle parameter from UI
            try:
                if hasattr(self, 'arc_param_var') and self.arc_param_var.get():
                    arc_angle = float(self.arc_param_var.get())
                else:
                    arc_angle = 90  # Default 90 degrees
            except ValueError:
                arc_angle = 90
            
            # Convert to radians
            arc_angle_rad = math.radians(arc_angle)
            
            # Convert robot heading to center-relative angle
            # Robot heading 0° = pointing right, but from center perspective
            if radius > 0:  # Forward - center is above robot
                start_angle = heading_rad + math.pi/2  # Convert to center-relative
            else:  # Backward - center is below robot  
                start_angle = heading_rad - math.pi/2  # Convert to center-relative
            
            # Debug: print what we're working with
            print(f"DEBUG: radius={radius}, arc_angle={arc_angle}")
            print(f"DEBUG: robot at ({robot_x:.1f}, {robot_y:.1f}), heading={robot_heading:.1f}°")
            print(f"DEBUG: center at ({center_x:.1f}, {center_y:.1f})")
            print(f"DEBUG: start_angle={math.degrees(start_angle):.1f}°")
            
            # Brute force casework - just make it work
            if radius > 0 and arc_angle > 0:  # Both positive: forward right
                end_angle = start_angle - arc_angle_rad
                print(f"DEBUG: Case 1 (both positive) - end_angle={math.degrees(end_angle):.1f}°")
            elif radius > 0 and arc_angle < 0:  # Radius positive, angle negative: forward left
                end_angle = start_angle + abs(arc_angle_rad)
                print(f"DEBUG: Case 2 (pos radius, neg angle) - end_angle={math.degrees(end_angle):.1f}°")
            elif radius < 0 and arc_angle > 0:  # Radius negative, angle positive: backward right
                end_angle = start_angle - arc_angle_rad
                print(f"DEBUG: Case 3 (neg radius, pos angle) - end_angle={math.degrees(end_angle):.1f}°")
            else:  # Both negative: backward left
                end_angle = start_angle + abs(arc_angle_rad)
                print(f"DEBUG: Case 4 (both negative) - end_angle={math.degrees(end_angle):.1f}°")
            
            # Draw the arc
            arc_angles = np.linspace(start_angle, end_angle, 50)
            arc_x = center_x + radius_cm * np.cos(arc_angles)
            arc_y = center_y + radius_cm * np.sin(arc_angles)
            
            self.ax.plot(arc_x, arc_y, 'r--', linewidth=2, alpha=0.8)
            
            # Also draw the robot's current position as a reference
            self.ax.plot(robot_x, robot_y, 'bo', markersize=8, alpha=0.8)
            
        except Exception as e:
            # Fallback to straight line if any calculation fails
            x_coords = [start[0], end[0]]
            y_coords = [start[1], end[1]]
            self.ax.plot(x_coords, y_coords, 'r--', linewidth=2, alpha=0.8)

    def load_field_image(self):
        """Load the WRO 2025 Junior field image with optimizations."""
        try:
            # Load image with PIL for better control
            from PIL import Image
            import numpy as np
            
            # Load the image
            pil_image = Image.open('wro2025_junior_field.png')
            
            # Resize to reduce memory usage and improve performance
            # Calculate appropriate size (maintain aspect ratio)
            max_size = 800  # Maximum dimension
            width, height = pil_image.size
            if width > max_size or height > max_size:
                if width > height:
                    new_width = max_size
                    new_height = int(height * max_size / width)
                else:
                    new_height = max_size
                    new_width = int(width * max_size / height)
                pil_image = pil_image.resize((new_width, new_height), Image.Resampling.LANCZOS)
            
            # Convert to numpy array
            self.field_image = np.array(pil_image)
            
        except Exception as e:
            print(f"Could not load field image: {e}")
            self.field_image = None

    def extract_waypoints_from_code(self):
        """Extract waypoints from generated code and create a waypoint editor."""
        # First generate the current code
        if not self.planner.commands:
            messagebox.showwarning("Warning", "No path to extract waypoints from!")
            return
            
        # Generate code to get the function
        code = self.planner.generate_code("extracted_path")
        
        # Create a waypoint editor with the current path
        self.create_waypoint_editor()
        
    def create_waypoint_editor(self):
        """Create a waypoint editor window."""
        # Create a new window for waypoint editing
        waypoint_window = tk.Toplevel(self.root)
        waypoint_window.title("Waypoint Editor")
        waypoint_window.geometry("1200x800")
        
        # Create a simplified waypoint editor interface
        self.create_simple_waypoint_editor(waypoint_window)
        
    def create_simple_waypoint_editor(self, parent, external_waypoints=None):
        """Create a simplified waypoint editor interface."""
        # Main frame
        main_frame = ttk.Frame(parent)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left panel for controls
        left_panel = ttk.Frame(main_frame, width=300)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # Right panel for plot
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Create plot
        fig = Figure(figsize=(10, 8))
        canvas = FigureCanvasTkAgg(fig, right_panel)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add navigation toolbar
        toolbar = NavigationToolbar2Tk(canvas, right_panel)
        toolbar.update()
        
        # Get waypoints from current path or use external waypoints
        if external_waypoints is not None:
            waypoints = external_waypoints
        else:
            waypoints = self.get_waypoints_from_path()
        
        # Waypoint list
        ttk.Label(left_panel, text="Waypoints", font=('Arial', 12, 'bold')).pack(pady=(0, 10))
        
        # Create listbox with scrollbar
        list_frame = ttk.Frame(left_panel)
        list_frame.pack(fill=tk.BOTH, expand=True)
        
        waypoint_listbox = tk.Listbox(list_frame, height=15)
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=waypoint_listbox.yview)
        waypoint_listbox.configure(yscrollcommand=scrollbar.set)
        
        waypoint_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Update waypoint list
        self.update_waypoint_list(waypoint_listbox, waypoints)
        
        # Buttons frame
        buttons_frame = ttk.Frame(left_panel)
        buttons_frame.pack(fill=tk.X, pady=(10, 0))
        
        ttk.Button(buttons_frame, text="Add Waypoint", 
                  command=lambda: self.add_waypoint_dialog(parent, waypoint_listbox, waypoints, fig, canvas)).pack(fill=tk.X, pady=2)
        ttk.Button(buttons_frame, text="Edit Waypoint", 
                  command=lambda: self.edit_waypoint_dialog(parent, waypoint_listbox, waypoints, fig, canvas)).pack(fill=tk.X, pady=2)
        ttk.Button(buttons_frame, text="Delete Waypoint", 
                  command=lambda: self.delete_waypoint(parent, waypoint_listbox, waypoints, fig, canvas)).pack(fill=tk.X, pady=2)
        ttk.Button(buttons_frame, text="Generate Code", 
                  command=lambda: self.generate_code_from_waypoints_dialog(parent, waypoints)).pack(fill=tk.X, pady=2)
        
        # Status bar
        status_frame = ttk.Frame(parent)
        status_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=(0, 10))
        
        status_label = ttk.Label(status_frame, text=f"Loaded {len(waypoints)} waypoints from path")
        status_label.pack(side=tk.LEFT)
        
        # Initial plot
        self.plot_waypoints(fig, canvas, waypoints)
        
    def get_waypoints_from_path(self):
        """Extract waypoints from the current path."""
        waypoints = []
        for i, (x, y, heading) in enumerate(self.planner.path_history):
            waypoints.append({
                'index': i,
                'x': x,
                'y': y,
                'heading': heading,
                'command': self.planner.commands[i] if i < len(self.planner.commands) else None
            })
        return waypoints
        
    def update_waypoint_list(self, listbox, waypoints):
        """Update the waypoint listbox."""
        listbox.delete(0, tk.END)
        for i, wp in enumerate(waypoints):
            display_text = f"WP {i}: ({wp['x']:.1f}, {wp['y']:.1f}) @ {wp['heading']:.1f}°"
            if wp['command']:
                cmd_type = wp['command']['type']
                display_text += f" - {cmd_type}"
            listbox.insert(tk.END, display_text)
            
    def plot_waypoints(self, fig, canvas, waypoints):
        """Plot waypoints on the figure."""
        ax = fig.add_subplot(111)
        ax.clear()
        
        # Set up the plot
        ax.set_aspect('equal')
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_title('Waypoint Editor')
        
        # Draw field boundaries
        field_rect = patches.Rectangle((0, 0), self.planner.field_width, self.planner.field_height, 
                                     linewidth=2, edgecolor='black', facecolor='none')
        ax.add_patch(field_rect)
        
        # Draw grid
        for x in range(0, int(self.planner.field_width) + 1, 10):
            ax.axvline(x=x, color='gray', alpha=0.3, linewidth=0.5)
        for y in range(0, int(self.planner.field_height) + 1, 10):
            ax.axhline(y=y, color='gray', alpha=0.3, linewidth=0.5)
        
        # Draw waypoints and path
        if len(waypoints) > 1:
            x_coords = [wp['x'] for wp in waypoints]
            y_coords = [wp['y'] for wp in waypoints]
            ax.plot(x_coords, y_coords, 'b-', linewidth=2, alpha=0.7, label='Robot Path')
            
            # Draw arrows to show direction
            for i, wp in enumerate(waypoints[:-1]):
                x, y, heading = wp['x'], wp['y'], wp['heading']
                dx = math.cos(math.radians(heading)) * 5
                dy = math.sin(math.radians(heading)) * 5
                ax.arrow(x, y, dx, dy, head_width=2, head_length=2, fc='blue', ec='blue', alpha=0.7)
        
        # Draw waypoints
        for i, wp in enumerate(waypoints):
            x, y, heading = wp['x'], wp['y'], wp['heading']
            
            # Draw waypoint marker
            ax.plot(x, y, 'o', color='green', markersize=8, markeredgecolor='black', markeredgewidth=2)
            
            # Draw robot at waypoint
            robot_rect = patches.Rectangle(
                (x - self.robot_size/2, y - self.robot_size/2),
                self.robot_size, self.robot_size,
                angle=heading,
                linewidth=1, edgecolor='green', facecolor='green', alpha=0.3
            )
            ax.add_patch(robot_rect)
            
            # Draw heading indicator
            dx = math.cos(math.radians(heading)) * (self.robot_size/2 + 2)
            dy = math.sin(math.radians(heading)) * (self.robot_size/2 + 2)
            ax.arrow(x, y, dx, dy, head_width=1, head_length=1, fc='green', ec='green')
            
            # Add waypoint number
            ax.annotate(f'{i}', (x, y), xytext=(5, 5), 
                       textcoords='offset points', fontsize=10,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
        
        # Set plot limits
        margin = 20
        ax.set_xlim(-margin, self.planner.field_width + margin)
        ax.set_ylim(-margin, self.planner.field_height + margin)
        
        # Add legend
        ax.legend()
        
        canvas.draw()
        
    def add_waypoint_dialog(self, parent, listbox, waypoints, fig, canvas):
        """Dialog for adding a new waypoint."""
        dialog = tk.Toplevel(parent)
        dialog.title("Add Waypoint")
        dialog.geometry("300x200")
        dialog.transient(parent)
        dialog.grab_set()
        
        # Center dialog
        dialog.geometry("+%d+%d" % (parent.winfo_rootx() + 50, parent.winfo_rooty() + 50))
        
        # Create widgets
        ttk.Label(dialog, text="X coordinate (cm):").pack(pady=5)
        x_entry = ttk.Entry(dialog)
        x_entry.pack(pady=5)
        x_entry.insert(0, "0")
        
        ttk.Label(dialog, text="Y coordinate (cm):").pack(pady=5)
        y_entry = ttk.Entry(dialog)
        y_entry.pack(pady=5)
        y_entry.insert(0, "0")
        
        ttk.Label(dialog, text="Heading (degrees):").pack(pady=5)
        heading_entry = ttk.Entry(dialog)
        heading_entry.pack(pady=5)
        heading_entry.insert(0, "0")
        
        # Buttons
        button_frame = ttk.Frame(dialog)
        button_frame.pack(pady=20)
        
        def add():
            try:
                x = float(x_entry.get())
                y = float(y_entry.get())
                heading = float(heading_entry.get())
                new_waypoint = {
                    'index': len(waypoints),
                    'x': x,
                    'y': y,
                    'heading': heading,
                    'command': None
                }
                waypoints.append(new_waypoint)
                self.update_waypoint_list(listbox, waypoints)
                self.plot_waypoints(fig, canvas, waypoints)
                dialog.destroy()
            except ValueError:
                messagebox.showerror("Error", "Please enter valid numbers.")
                
        ttk.Button(button_frame, text="Add", command=add).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=dialog.destroy).pack(side=tk.RIGHT, padx=5)
        
    def edit_waypoint_dialog(self, parent, listbox, waypoints, fig, canvas):
        """Dialog for editing an existing waypoint."""
        selection = listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a waypoint to edit.")
            return
            
        index = selection[0]
        wp = waypoints[index]
        
        dialog = tk.Toplevel(parent)
        dialog.title("Edit Waypoint")
        dialog.geometry("300x200")
        dialog.transient(parent)
        dialog.grab_set()
        
        # Center dialog
        dialog.geometry("+%d+%d" % (parent.winfo_rootx() + 50, parent.winfo_rooty() + 50))
        
        # Create widgets
        ttk.Label(dialog, text="X coordinate (cm):").pack(pady=5)
        x_entry = ttk.Entry(dialog)
        x_entry.pack(pady=5)
        x_entry.insert(0, str(wp['x']))
        
        ttk.Label(dialog, text="Y coordinate (cm):").pack(pady=5)
        y_entry = ttk.Entry(dialog)
        y_entry.pack(pady=5)
        y_entry.insert(0, str(wp['y']))
        
        ttk.Label(dialog, text="Heading (degrees):").pack(pady=5)
        heading_entry = ttk.Entry(dialog)
        heading_entry.pack(pady=5)
        heading_entry.insert(0, str(wp['heading']))
        
        # Buttons
        button_frame = ttk.Frame(dialog)
        button_frame.pack(pady=20)
        
        def update():
            try:
                x = float(x_entry.get())
                y = float(y_entry.get())
                heading = float(heading_entry.get())
                wp['x'] = x
                wp['y'] = y
                wp['heading'] = heading
                self.update_waypoint_list(listbox, waypoints)
                self.plot_waypoints(fig, canvas, waypoints)
                dialog.destroy()
            except ValueError:
                messagebox.showerror("Error", "Please enter valid numbers.")
                
        ttk.Button(button_frame, text="Update", command=update).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=dialog.destroy).pack(side=tk.RIGHT, padx=5)
        
    def delete_waypoint(self, parent, listbox, waypoints, fig, canvas):
        """Delete the selected waypoint."""
        selection = listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a waypoint to delete.")
            return
            
        if messagebox.askyesno("Confirm Delete", "Are you sure you want to delete this waypoint?"):
            index = selection[0]
            del waypoints[index]
            # Update indices
            for i, wp in enumerate(waypoints):
                wp['index'] = i
            self.update_waypoint_list(listbox, waypoints)
            self.plot_waypoints(fig, canvas, waypoints)
            
    def generate_code_from_waypoints_dialog(self, parent, waypoints):
        """Generate code from waypoints and show in dialog."""
        if not waypoints:
            messagebox.showwarning("Warning", "No waypoints to generate code from!")
            return
            
        # Create a new path planner with the waypoints
        new_planner = PathPlanner()
        if waypoints:
            new_planner.reset_position(waypoints[0]['x'], waypoints[0]['y'], waypoints[0]['heading'])
            
        # Generate code
        code = new_planner.generate_code("edited_path")
        
        # Show code in dialog
        code_window = tk.Toplevel(parent)
        code_window.title("Generated Code from Waypoints")
        code_window.geometry("800x600")
        
        # Add text widget
        text_widget = tk.Text(code_window, wrap=tk.WORD, font=("Courier", 10))
        text_widget.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Add scrollbar
        scrollbar = ttk.Scrollbar(code_window, orient=tk.VERTICAL, command=text_widget.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        text_widget.configure(yscrollcommand=scrollbar.set)
        
        # Insert code
        text_widget.insert(tk.END, code)
        text_widget.config(state=tk.DISABLED)
        
        # Add copy button
        def copy_code():
            code_window.clipboard_clear()
            code_window.clipboard_append(code)
            messagebox.showinfo("Success", "Code copied to clipboard!")
        ttk.Button(code_window, text="Copy to Clipboard", command=copy_code).pack(pady=10)
        
    def run_program_and_extract(self):
        """Run a program and extract waypoints from its execution."""
        # Ask user to select a Python file to run
        filename = filedialog.askopenfilename(
            title="Select Python file to run",
            filetypes=[("Python files", "*.py"), ("All files", "*.*")]
        )
        
        if not filename:
            return
            
        try:
            # Read the file content
            with open(filename, 'r') as f:
                file_content = f.read()
            
            # Create a dialog to show the file content and allow editing
            self.show_program_editor_dialog(file_content, filename)
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to read the file:\n{str(e)}")
            
    def show_program_editor_dialog(self, file_content, filename):
        """Show a dialog to edit and run the program."""
        dialog = tk.Toplevel(self.root)
        dialog.title(f"Program Editor - {filename}")
        dialog.geometry("1000x700")
        dialog.transient(self.root)
        
        # Main frame
        main_frame = ttk.Frame(dialog)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Top frame for buttons
        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(top_frame, text="Run Program", 
                  command=lambda: self.execute_program_and_extract(file_content, dialog)).pack(side=tk.LEFT, padx=5)
        ttk.Button(top_frame, text="Extract Waypoints from Code", 
                  command=lambda: self.extract_waypoints_from_program_code(file_content, dialog)).pack(side=tk.LEFT, padx=5)
        ttk.Button(top_frame, text="Close", command=dialog.destroy).pack(side=tk.RIGHT, padx=5)
        
        # Text frame
        text_frame = ttk.Frame(main_frame)
        text_frame.pack(fill=tk.BOTH, expand=True)
        
        # Text widget with scrollbar
        text_widget = tk.Text(text_frame, wrap=tk.NONE, font=("Courier", 10))
        scrollbar_y = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=text_widget.yview)
        scrollbar_x = ttk.Scrollbar(text_frame, orient=tk.HORIZONTAL, command=text_widget.xview)
        text_widget.configure(yscrollcommand=scrollbar_y.set, xscrollcommand=scrollbar_x.set)
        
        text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar_y.pack(side=tk.RIGHT, fill=tk.Y)
        scrollbar_x.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Insert file content
        text_widget.insert(tk.END, file_content)
        
        # Status bar
        status_frame = ttk.Frame(dialog)
        status_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=(0, 10))
        
        status_label = ttk.Label(status_frame, text="Ready to run program or extract waypoints")
        status_label.pack(side=tk.LEFT)
        
        # Store references for callbacks
        self.current_dialog_status = status_label
        self.current_dialog_text = text_widget
        
    def execute_program_and_extract(self, code_content, dialog):
        """Execute the program and extract waypoints."""
        try:
            self.current_dialog_status.config(text="Executing program...")
            dialog.update()
            
            # Create a temporary file with the code
            import tempfile
            import os
            
            with tempfile.NamedTemporaryFile('w', suffix='.py', delete=False) as tmp:
                tmp.write(code_content)
                tmp_path = tmp.name
            
            try:
                # Import and execute the program
                import importlib.util
                
                spec = importlib.util.spec_from_file_location("program_module", tmp_path)
                if spec is None or spec.loader is None:
                    raise ImportError("Could not load the program.")
                    
                program_module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(program_module)
                
                # Look for path functions
                path_functions = []
                for attr_name in dir(program_module):
                    if attr_name.endswith('_path') or attr_name == 'main':
                        attr = getattr(program_module, attr_name)
                        if callable(attr):
                            path_functions.append((attr_name, attr))
                
                if not path_functions:
                    self.current_dialog_status.config(text="No path functions found")
                    messagebox.showwarning("Warning", "No path functions found in the program.")
                    return
                
                # Execute each path function and capture waypoints
                all_waypoints = []
                for func_name, func in path_functions:
                    try:
                        # Create a new path planner for this execution
                        execution_planner = PathPlanner()
                        
                        # Inject the path planner into the module
                        setattr(program_module, 'PathPlanner', PathPlanner)
                        
                        # Execute the function
                        func()
                        
                        # Extract waypoints from the execution
                        waypoints = self.get_waypoints_from_planner(execution_planner)
                        all_waypoints.extend(waypoints)
                        
                        self.current_dialog_status.config(text=f"Executed {func_name}, found {len(waypoints)} waypoints")
                        
                    except Exception as e:
                        self.current_dialog_status.config(text=f"Error executing {func_name}: {str(e)}")
                        messagebox.showerror("Execution Error", f"Error executing {func_name}:\n{str(e)}")
                
                # Show waypoint editor with extracted waypoints
                if all_waypoints:
                    self.show_waypoint_editor_with_waypoints(all_waypoints)
                    self.current_dialog_status.config(text=f"Extracted {len(all_waypoints)} total waypoints")
                else:
                    self.current_dialog_status.config(text="No waypoints extracted")
                    
            finally:
                # Clean up temporary file
                try:
                    os.remove(tmp_path)
                except:
                    pass
                    
        except Exception as e:
            self.current_dialog_status.config(text=f"Error: {str(e)}")
            messagebox.showerror("Error", f"Failed to execute program:\n{str(e)}")
            
    def extract_waypoints_from_program_code(self, code_content, dialog):
        """Extract waypoints by analyzing the program code."""
        try:
            self.current_dialog_status.config(text="Analyzing program code...")
            dialog.update()
            
            # Create a path planner to simulate the robot movements
            simulation_planner = PathPlanner()
            
            # Parse the code and execute commands to build the path
            waypoints = self.parse_and_simulate_robot_code(code_content, simulation_planner)
            
            if waypoints:
                self.show_waypoint_editor_with_waypoints(waypoints)
                self.current_dialog_status.config(text=f"Extracted {len(waypoints)} waypoints from code analysis")
            else:
                self.current_dialog_status.config(text="No waypoints found in code analysis")
                messagebox.showinfo("Info", "No waypoints could be extracted from the code analysis.")
                
        except Exception as e:
            self.current_dialog_status.config(text=f"Error: {str(e)}")
            messagebox.showerror("Error", f"Failed to analyze code:\n{str(e)}")
            
    def parse_and_simulate_robot_code(self, code_content, planner):
        """Parse robot code and simulate movements to extract waypoints."""
        import re
        
        # Reset planner to starting position
        planner.reset_position(0, 0, 0)  # Default start, will be updated if found
        
        lines = code_content.split('\n')
        waypoints = []
        
        for i, line in enumerate(lines):
            line = line.strip()
            
            # Skip comments and empty lines
            if line.startswith('#') or not line:
                continue
                
            # Look for heading reset
            if 'hub.imu.reset_heading(' in line:
                match = re.search(r'hub\.imu\.reset_heading\(([0-9.-]+)\)', line)
                if match:
                    heading = float(match.group(1))
                    planner.heading = heading
                    planner.x = 0
                    planner.y = 0
                    planner.path_history = [(0, 0, heading)]
                    waypoints.append({
                        'index': len(waypoints),
                        'x': 0,
                        'y': 0,
                        'heading': heading,
                        'command': {'type': 'reset_heading', 'line': i+1, 'value': heading}
                    })
                    continue
            
            # Look for gyroStraightWithDrive commands
            if 'gyroStraightWithDrive(' in line:
                waypoint = self.parse_gyro_straight_command(line, planner, i+1)
                if waypoint:
                    waypoints.append(waypoint)
                    continue
            
            # Look for turnToAngle commands
            if 'turnToAngle(' in line:
                waypoint = self.parse_turn_command(line, planner, i+1)
                if waypoint:
                    waypoints.append(waypoint)
                    continue
            
            # Look for curve commands
            if 'curve(' in line and 'curve_with_accel_options(' not in line:
                waypoint = self.parse_curve_command(line, planner, i+1)
                if waypoint:
                    waypoints.append(waypoint)
                    continue
            
            # Look for curve_with_accel_options commands
            if 'curve_with_accel_options(' in line:
                waypoint = self.parse_curve_accel_command(line, planner, i+1)
                if waypoint:
                    waypoints.append(waypoint)
                    continue
            
            # Look for drive_base.straight commands
            if 'drive_base.straight(' in line:
                waypoint = self.parse_drive_straight_command(line, planner, i+1)
                if waypoint:
                    waypoints.append(waypoint)
                    continue
            
            # Look for drive_base.turn commands
            if 'drive_base.turn(' in line:
                waypoint = self.parse_drive_turn_command(line, planner, i+1)
                if waypoint:
                    waypoints.append(waypoint)
                    continue
            
            # Look for setPickersPosition commands
            if 'setPickersPosition(' in line:
                waypoint = self.parse_picker_command(line, planner, i+1)
                if waypoint:
                    waypoints.append(waypoint)
                    continue
            
            # Look for setBallPickerPosition commands
            if 'setBallPickerPosition(' in line:
                waypoint = self.parse_ball_picker_command(line, planner, i+1)
                if waypoint:
                    waypoints.append(waypoint)
                    continue
            
            # Look for wait commands
            if 'wait(' in line:
                waypoint = self.parse_wait_command(line, planner, i+1)
                if waypoint:
                    waypoints.append(waypoint)
                    continue
        
        return waypoints
    
    def parse_gyro_straight_command(self, line, planner, line_num):
        """Parse gyroStraightWithDrive command and update planner."""
        import re
        
        # Extract parameters
        distance_match = re.search(r'distance=([0-9.-]+)', line)
        speed_match = re.search(r'speed=([0-9.-]+)', line)
        target_angle_match = re.search(r'targetAngle=([0-9.-]+)', line)
        backward_match = re.search(r'backward=True', line)
        till_black_match = re.search(r'tillBlackLine=True', line)
        till_white_match = re.search(r'tillWhiteLine=True', line)
        slow_down_match = re.search(r'slowDown=False', line)
        
        if distance_match:
            distance = float(distance_match.group(1))
            speed = float(speed_match.group(1)) if speed_match else 400
            target_angle = float(target_angle_match.group(1)) if target_angle_match else None
            backward = bool(backward_match)
            till_black = bool(till_black_match)
            till_white = bool(till_white_match)
            slow_down = not bool(slow_down_match) if slow_down_match else True
            
            # Update planner
            planner.drive_straight(
                distance=distance,
                speed=speed,
                backward=backward,
                target_angle=target_angle,
                till_black_line=till_black,
                till_white_line=till_white,
                slow_down=slow_down
            )
            
            return {
                'index': len(planner.path_history) - 1,
                'x': planner.x,
                'y': planner.y,
                'heading': planner.heading,
                'command': {
                    'type': 'gyroStraightWithDrive',
                    'line': line_num,
                    'distance': distance,
                    'speed': speed,
                    'backward': backward,
                    'target_angle': target_angle
                }
            }
        return None
    
    def parse_turn_command(self, line, planner, line_num):
        """Parse turnToAngle command and update planner."""
        import re
        
        target_angle_match = re.search(r'targetAngle=([0-9.-]+)', line)
        speed_match = re.search(r'speed=([0-9.-]+)', line)
        one_wheel_match = re.search(r'oneWheelTurn=True', line)
        force_turn_match = re.search(r'forceTurn=([A-Z_]+)', line)
        
        if target_angle_match:
            target_angle = float(target_angle_match.group(1))
            speed = float(speed_match.group(1)) if speed_match else 400
            one_wheel = bool(one_wheel_match)
            force_turn = force_turn_match.group(1) if force_turn_match else None
            
            # Update planner
            planner.turn_to_angle(
                target_angle=target_angle,
                speed=speed,
                one_wheel_turn=one_wheel,
                force_turn=force_turn
            )
            
            return {
                'index': len(planner.path_history) - 1,
                'x': planner.x,
                'y': planner.y,
                'heading': planner.heading,
                'command': {
                    'type': 'turnToAngle',
                    'line': line_num,
                    'target_angle': target_angle,
                    'speed': speed
                }
            }
        return None
    
    def parse_curve_command(self, line, planner, line_num):
        """Parse curve command and update planner."""
        import re
        radius_match = re.search(r'radius=([0-9.-]+)', line)
        angle_match = re.search(r'angle=([0-9.-]+)', line)
        speed_match = re.search(r'speed=([0-9.-]+)', line)
        if radius_match and angle_match:
            radius = float(radius_match.group(1)) / 10  # mm to cm
            angle = float(angle_match.group(1))
            speed = float(speed_match.group(1)) if speed_match else 300
            planner.curve(radius=radius, angle=angle, speed=speed)
            return {
                'index': len(planner.path_history) - 1,
                'x': planner.x,
                'y': planner.y,
                'heading': planner.heading,
                'command': {
                    'type': 'curve',
                    'line': line_num,
                    'radius': radius,
                    'angle': angle,
                    'speed': speed
                }
            }
        return None
    
    def parse_curve_accel_command(self, line, planner, line_num):
        """Parse curve_with_accel_options command and update planner."""
        import re
        radius_match = re.search(r'radius=([0-9.-]+)', line)
        angle_match = re.search(r'angle=([0-9.-]+)', line)
        speed_match = re.search(r'speed=([0-9.-]+)', line)
        dont_accel_match = re.search(r'dont_accelerate=True', line)
        dont_decel_match = re.search(r'dont_decelerate=True', line)
        if radius_match and angle_match:
            radius = float(radius_match.group(1)) / 10  # mm to cm
            angle = float(angle_match.group(1))
            speed = float(speed_match.group(1)) if speed_match else 300
            dont_accel = bool(dont_accel_match)
            dont_decel = bool(dont_decel_match)
            planner.curve(
                radius=radius, 
                angle=angle, 
                speed=speed,
                dont_accelerate=dont_accel,
                dont_decelerate=dont_decel
            )
            return {
                'index': len(planner.path_history) - 1,
                'x': planner.x,
                'y': planner.y,
                'heading': planner.heading,
                'command': {
                    'type': 'curve_with_accel_options',
                    'line': line_num,
                    'radius': radius,
                    'angle': angle,
                    'speed': speed
                }
            }
        return None
    
    def parse_drive_straight_command(self, line, planner, line_num):
        """Parse drive_base.straight command and update planner."""
        import re
        distance_match = re.search(r'drive_base\.straight\(([0-9.-]+)\)', line)
        if distance_match:
            distance = float(distance_match.group(1)) / 10  # mm to cm
            speed = 400  # Default speed
            planner.drive_straight(distance=distance, speed=speed)
            return {
                'index': len(planner.path_history) - 1,
                'x': planner.x,
                'y': planner.y,
                'heading': planner.heading,
                'command': {
                    'type': 'drive_base.straight',
                    'line': line_num,
                    'distance': distance * 10,  # Store in mm
                    'speed': speed
                }
            }
        return None
    
    def parse_drive_turn_command(self, line, planner, line_num):
        """Parse drive_base.turn command and update planner."""
        import re
        
        angle_match = re.search(r'drive_base\.turn\(([0-9.-]+)\)', line)
        
        if angle_match:
            angle = float(angle_match.group(1))
            speed = 400  # Default speed
            
            # Update planner
            planner.turn_to_angle(target_angle=angle, speed=speed)
            
            return {
                'index': len(planner.path_history) - 1,
                'x': planner.x,
                'y': planner.y,
                'heading': planner.heading,
                'command': {
                    'type': 'drive_base.turn',
                    'line': line_num,
                    'angle': angle,
                    'speed': speed
                }
            }
        return None
    
    def parse_picker_command(self, line, planner, line_num):
        """Parse setPickersPosition command and update planner."""
        import re
        
        position_match = re.search(r'setPickersPosition\(([A-Z_]+)', line)
        speed_match = re.search(r'speed=([0-9.-]+)', line)
        wait_match = re.search(r'wait=False', line)
        
        if position_match:
            position = position_match.group(1)
            speed = float(speed_match.group(1)) if speed_match else 700
            wait = not bool(wait_match)
            
            # Update planner
            planner.set_pickers_position(position=position, speed=speed, wait=wait)
            
            return {
                'index': len(planner.path_history) - 1,
                'x': planner.x,
                'y': planner.y,
                'heading': planner.heading,
                'command': {
                    'type': 'setPickersPosition',
                    'line': line_num,
                    'position': position,
                    'speed': speed,
                    'wait': wait
                }
            }
        return None
    
    def parse_ball_picker_command(self, line, planner, line_num):
        """Parse setBallPickerPosition command and update planner."""
        import re
        
        position_match = re.search(r'setBallPickerPosition\(([A-Z_]+)', line)
        speed_match = re.search(r'speed=([0-9.-]+)', line)
        wait_match = re.search(r'wait=False', line)
        
        if position_match:
            position = position_match.group(1)
            speed = float(speed_match.group(1)) if speed_match else 1000
            wait = not bool(wait_match)
            
            # Update planner
            planner.set_ball_picker_position(position=position, speed=speed, wait=wait)
            
            return {
                'index': len(planner.path_history) - 1,
                'x': planner.x,
                'y': planner.y,
                'heading': planner.heading,
                'command': {
                    'type': 'setBallPickerPosition',
                    'line': line_num,
                    'position': position,
                    'speed': speed,
                    'wait': wait
                }
            }
        return None
    
    def parse_wait_command(self, line, planner, line_num):
        """Parse wait command and update planner."""
        import re
        
        time_match = re.search(r'wait\(([0-9.-]+)\)', line)
        
        if time_match:
            wait_time = int(time_match.group(1))
            
            # Update planner
            planner.wait(wait_time)
            
            return {
                'index': len(planner.path_history) - 1,
                'x': planner.x,
                'y': planner.y,
                'heading': planner.heading,
                'command': {
                    'type': 'wait',
                    'line': line_num,
                    'milliseconds': wait_time
                }
            }
        return None
            
    def get_waypoints_from_planner(self, planner):
        """Extract waypoints from a path planner instance."""
        waypoints = []
        for i, (x, y, heading) in enumerate(planner.path_history):
            waypoints.append({
                'index': i,
                'x': x,
                'y': y,
                'heading': heading,
                'command': planner.commands[i] if i < len(planner.commands) else None
            })
        return waypoints
        
    def show_waypoint_editor_with_waypoints(self, waypoints):
        """Show waypoint editor with the given waypoints."""
        # Create a new window for waypoint editing
        waypoint_window = tk.Toplevel(self.root)
        waypoint_window.title("Waypoint Editor - Extracted Waypoints")
        waypoint_window.geometry("1200x800")
        
        # Create the waypoint editor interface with the extracted waypoints
        self.create_simple_waypoint_editor(waypoint_window, external_waypoints=waypoints)
        
        # Show success message
        messagebox.showinfo("Success", f"Extracted {len(waypoints)} waypoints!\nUse the waypoint editor to modify them.")
        
    def edit_waypoints(self):
        """Open waypoint editor for the current path."""
        if not self.planner.commands:
            messagebox.showwarning("Warning", "No path to edit waypoints from!")
            return
            
        # Create waypoint editor
        self.create_waypoint_editor()
        
    def generate_code_from_waypoints(self):
        """Generate code from waypoints."""
        if not self.planner.commands:
            messagebox.showwarning("Warning", "No path to generate code from!")
            return
            
        # Create waypoint editor first
        self.create_waypoint_editor()


def main():
    """Main function to run the path drawer."""
    root = tk.Tk()
    app = PathDrawer(root)
    root.mainloop()


if __name__ == "__main__":
    main() 