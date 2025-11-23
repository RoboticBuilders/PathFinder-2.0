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
import json
from matplotlib.figure import Figure
from matplotlib.transforms import Affine2D
import numpy as np
import math
from typing import List, Tuple, Dict, Optional
import json
from PIL import Image 
import os

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
        # Load extracted missions if available to integrate with guidelines
        self.load_missions()
        
        # Set initial position based on selected start position
        x, y, heading = self.get_start_position()
        self.planner.reset_position(x, y, heading)
        
        # Initialize plot after UI setup to avoid None errors
        if self.ax is not None:
            self.update_plot()
        # Pasted code block (appended to generated code)
        self.pasted_code = ""
        
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
            ("Set Right Pickers", "right_pickers"),
            ("Set Left Pickers", "left_pickers"),
            ("Set Backarm", "backarm"),
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
        ttk.Button(actions_frame, text="Load Missions", command=self.load_missions).pack(fill=tk.X, pady=2)
        ttk.Button(actions_frame, text="Paste Code", command=self.paste_code_dialog).pack(fill=tk.X, pady=2)
        ttk.Button(actions_frame, text="Save Missions", command=self.save_missions).pack(fill=tk.X, pady=2)
        
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

        # Edit missions checkbox (enable dragging mission bounding boxes)
        self.edit_missions_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(robot_frame, text="Edit Missions (drag boxes)", 
                   variable=self.edit_missions_var).pack(anchor=tk.W)
        
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
            self.speed_var = tk.StringVar(value="400")
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
            self.speed_var = tk.StringVar(value="400")
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
            
        elif self.mode_var.get() == "right_pickers":
            # Position - Updated to new arm positions/functions (right/left pickers)
            positions = [
                "ALL_UP",
                "DROP_BLOCKS",
                "PICK_BLOCKS",
                "HOLD_BLOCKS",
                "PICK_DRONE"
            ]
            ttk.Label(self.params_frame, text="Position:").pack(anchor=tk.W)
            self.picker_position_var = tk.StringVar(value="ALL_UP")
            picker_combo = ttk.Combobox(self.params_frame, textvariable=self.picker_position_var, values=positions)
            picker_combo.pack(fill=tk.X, pady=(0, 10))

            # Speed
            ttk.Label(self.params_frame, text="Speed:").pack(anchor=tk.W)
            self.speed_var = tk.StringVar(value="600")
            ttk.Entry(self.params_frame, textvariable=self.speed_var).pack(fill=tk.X, pady=(0, 10))
            # (No custom angle control)
            
        elif self.mode_var.get() == "left_pickers":
            # Left pickers positions
            positions = [
                "ALL_UP",
                "PICK_BLOCKS",
                "HOLD_BLOCKS",
                "ROVER",
                "DROP_BLOCKS"
            ]
            ttk.Label(self.params_frame, text="Position:").pack(anchor=tk.W)
            self.left_picker_position_var = tk.StringVar(value="HOLD_BLOCKS")
            left_combo = ttk.Combobox(self.params_frame, textvariable=self.left_picker_position_var, values=positions)
            left_combo.pack(fill=tk.X, pady=(0, 10))

            # Speed
            ttk.Label(self.params_frame, text="Speed:").pack(anchor=tk.W)
            self.speed_var = tk.StringVar(value="600")
            ttk.Entry(self.params_frame, textvariable=self.speed_var).pack(fill=tk.X, pady=(0, 10))
            # (No custom angle control)

        elif self.mode_var.get() == "backarm":
            # Position - Updated to new back-arm positions/functions
            positions = [
                "PICK_BALLS",
                "DROP_BALLS",
                "SLIDE_BALL",
                "PICK_BLOCKS_BACKARM",
                "HOLD_BLOCKS_BACKARM",
                "RYGW_SWAP_PICK"
            ]
            ttk.Label(self.params_frame, text="Position:").pack(anchor=tk.W)
            self.ball_picker_position_var = tk.StringVar(value="PICK_BALLS")
            ball_combo = ttk.Combobox(self.params_frame, textvariable=self.ball_picker_position_var, values=positions)
            ball_combo.pack(fill=tk.X, pady=(0, 10))

            # Speed
            ttk.Label(self.params_frame, text="Speed:").pack(anchor=tk.W)
            self.speed_var = tk.StringVar(value="600")
            ttk.Entry(self.params_frame, textvariable=self.speed_var).pack(fill=tk.X, pady=(0, 10))
            # (No custom angle control)
            
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
        # Mission drag state
        self.mission_dragging = False
        self._drag_mission_index = None
        self._drag_offset = (0.0, 0.0)
        # Rotation state
        self.mission_rotating = False
        self._rotate_mission_index = None
        self._rotate_start_pointer_angle = 0.0
        self._rotate_start_mission_angle = 0.0
        # Selected mission (for keyboard rotation)
        self.selected_mission_index = None
        
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

        # Draw mission bounding boxes (configurable). If `self.missions` isn't
        # defined, create a set of reasonable defaults positioned on the field.
        # Each mission is a dict: {'name','x','y','w','h','color'}
        if not hasattr(self, 'missions') or not self.missions:
            fw = self.planner.field_width
            fh = self.planner.field_height
            # Default boxes extracted from the field image (auto-extracted and
            # saved to `tools/extracted_missions.json`). Coordinates are in cm
            # with origin at bottom-left, matching the planner plotting system.
            self.missions = [
                {'name': 'M3',  'x': 12.8,  'y': 88.5,  'w': 8.2,  'h': 9.0,  'color': 'purple'},
                {'name': 'M4',  'x': 76.4,  'y': 43.1,  'w': 6.6,  'h': 12.1, 'color': 'cyan'},
                {'name': 'M7',  'x': 140.5, 'y': 14.5,  'w': 22.2, 'h': 16.8, 'color': 'brown'},
                {'name': 'M10', 'x': 198.3, 'y': 74.0,  'w': 29.9, 'h': 2.6,  'color': 'grey'},
                {'name': 'M11', 'x': 198.3, 'y': 40.7,  'w': 29.9, 'h': 2.6,  'color': 'grey'},
                {'name': 'M12', 'x': 201.2, 'y': 45.8,  'w': 25.2, 'h': 25.1, 'color': 'red'},
                {'name': 'M13', 'x': 206.2, 'y': 96.4,  'w': 10.2, 'h': 10.2, 'color': 'pink'},
                {'name': 'M14', 'x': 206.2, 'y': 80.8,  'w': 10.2, 'h': 10.2, 'color': 'pink'},
                {'name': 'M15', 'x': 206.2, 'y': 23.7,  'w': 10.2, 'h': 10.2, 'color': 'pink'},
                {'name': 'M16', 'x': 206.2, 'y': 8.1,   'w': 10.2, 'h': 10.2, 'color': 'lime'}
            ]

        for m in self.missions:
            try:
                # Ensure angle exists
                angle = m.get('angle', 0.0)
                cx = m['x'] + m['w'] / 2.0
                cy = m['y'] + m['h'] / 2.0
                # Draw rectangle then rotate about its center
                rect = patches.Rectangle((m['x'], m['y']), m['w'], m['h'], linewidth=2,
                                         edgecolor=m.get('color', 'yellow'), facecolor=m.get('color', 'yellow'), alpha=0.12)
                trans = Affine2D().rotate_deg_around(cx, cy, angle) + self.ax.transData
                rect.set_transform(trans)
                # Highlight if selected
                idx = self.missions.index(m)
                if getattr(self, 'selected_mission_index', None) == idx:
                    rect.set_linewidth(3)
                    rect.set_alpha(0.18)
                    rect.set_edgecolor('black')
                self.ax.add_patch(rect)
                # Label the mission near the top-left of the (unrotated) box
                label_x = cx - m['w']/2 + 3
                label_y = cy + m['h']/2 - 3
                self.ax.annotate(m['name'], (label_x, label_y), fontsize=9,
                                 color=m.get('color', 'black'), weight='bold',
                                 bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.6))
            except Exception:
                # Ignore malformed mission entries
                pass

        # (Guidelines feature removed) — missions are still drawn as rotated rectangles
        
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
        elif event.key == 'r' or event.key == 'R':
            # Rotate selected mission via keyboard: 'r' = +15°, 'R' = -15°
            idx = getattr(self, 'selected_mission_index', None)
            if idx is None or idx < 0 or idx >= len(getattr(self, 'missions', [])):
                return
            m = self.missions[idx]
            step = 15.0
            if event.key == 'R':
                step = -15.0
            new_angle = (m.get('angle', 0.0) + step) % 360
            if new_angle > 180:
                new_angle -= 360
            m['angle'] = new_angle
            if hasattr(self, 'status_label'):
                self.status_label.config(text=f"Rotated {m['name']} to {m['angle']:.1f}° (keyboard)")
            self.update_plot()
            
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

        # If mission edit mode is enabled, check for clicks inside mission boxes
        if self.edit_missions_var.get() and event.button == 1:
            for i, m in enumerate(getattr(self, 'missions', [])):
                # mission coords are bottom-left origin
                if event.xdata is None or event.ydata is None:
                    continue
                if m['x'] <= event.xdata <= m['x'] + m['w'] and m['y'] <= event.ydata <= m['y'] + m['h']:
                    # Start dragging this mission
                    self.mission_dragging = True
                    self._drag_mission_index = i
                    self._drag_offset = (event.xdata - m['x'], event.ydata - m['y'])
                    # mark selection for keyboard rotation
                    self.selected_mission_index = i
                    if hasattr(self, 'status_label'):
                        self.status_label.config(text=f"Dragging mission {m['name']}")
                    return

        # If mission edit mode is enabled, right-click inside a mission to start rotating
        if self.edit_missions_var.get() and event.button == 3:
            for i, m in enumerate(getattr(self, 'missions', [])):
                if event.xdata is None or event.ydata is None:
                    continue
                # To test containment for rotated rectangles approximately, transform
                # the pointer back by the negative rotation and test against axis-aligned box
                angle = m.get('angle', 0.0)
                cx = m['x'] + m['w']/2.0
                cy = m['y'] + m['h']/2.0
                # rotate point around center by -angle
                rad = math.radians(-angle)
                dx = event.xdata - cx
                dy = event.ydata - cy
                rx = dx * math.cos(rad) - dy * math.sin(rad) + cx
                ry = dx * math.sin(rad) + dy * math.cos(rad) + cy
                if m['x'] <= rx <= m['x'] + m['w'] and m['y'] <= ry <= m['y'] + m['h']:
                    # start rotating
                    self.mission_rotating = True
                    self._rotate_mission_index = i
                    # store pointer angle and mission starting angle
                    self._rotate_start_pointer_angle = math.degrees(math.atan2(event.ydata - cy, event.xdata - cx))
                    self._rotate_start_mission_angle = m.get('angle', 0.0)
                    # mark selection for keyboard rotation
                    self.selected_mission_index = i
                    if hasattr(self, 'status_label'):
                        self.status_label.config(text=f"Rotating mission {m['name']}")
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
        # If dragging a mission, update its position
        if self.mission_dragging and event.inaxes == self.ax and self._drag_mission_index is not None:
            if event.xdata is None or event.ydata is None:
                return
            m = self.missions[self._drag_mission_index]
            ox, oy = self._drag_offset
            new_x = event.xdata - ox
            new_y = event.ydata - oy
            # Clamp to field bounds
            new_x = max(0, min(self.planner.field_width - m['w'], new_x))
            new_y = max(0, min(self.planner.field_height - m['h'], new_y))
            m['x'] = new_x
            m['y'] = new_y
            if hasattr(self, 'status_label'):
                self.status_label.config(text=f"Dragging {m['name']} to ({m['x']:.1f}, {m['y']:.1f})")
            self.update_plot()
            return
        # If rotating, update mission angle
        if self.mission_rotating and event.inaxes == self.ax and self._rotate_mission_index is not None:
            if event.xdata is None or event.ydata is None:
                return
            m = self.missions[self._rotate_mission_index]
            cx = m['x'] + m['w']/2.0
            cy = m['y'] + m['h']/2.0
            pointer_angle = math.degrees(math.atan2(event.ydata - cy, event.xdata - cx))
            delta = pointer_angle - self._rotate_start_pointer_angle
            new_angle = (self._rotate_start_mission_angle + delta) % 360
            # Normalize to [-180,180]
            if new_angle > 180:
                new_angle -= 360
            m['angle'] = new_angle
            if hasattr(self, 'status_label'):
                self.status_label.config(text=f"Rotating {m['name']} to {m['angle']:.1f}°")
            self.update_plot()
            return

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
        # If we were dragging a mission, finalize the move
        if self.mission_dragging:
            self.mission_dragging = False
            idx = self._drag_mission_index
            self._drag_mission_index = None
            self._drag_offset = (0.0, 0.0)
            if hasattr(self, 'status_label') and idx is not None:
                m = self.missions[idx]
                self.status_label.config(text=f"Moved {m['name']} to ({m['x']:.1f}, {m['y']:.1f})")
            # Redraw to show final position
            self.update_plot()
            return

        # If we were rotating a mission, finalize rotation
        if self.mission_rotating:
            self.mission_rotating = False
            ridx = self._rotate_mission_index
            self._rotate_mission_index = None
            if hasattr(self, 'status_label') and ridx is not None:
                m = self.missions[ridx]
                self.status_label.config(text=f"Rotated {m['name']} to {m.get('angle',0):.1f}°")
            self.update_plot()
            return

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
                
            elif mode == "right_pickers":
                pos_str = self.picker_position_var.get()
                speed = int(self.speed_var.get())
                # Map UI string to numeric constant
                RIGHT_MAP = {
                    'ALL_UP': 0,
                    'DROP_BLOCKS': 1,
                    'PICK_BLOCKS': 2,
                    'HOLD_BLOCKS': 3,
                    'PICK_DRONE': 4
                }
                position = RIGHT_MAP.get(pos_str, 0)
                self.planner.setRightPickersPosition(
                    position=position,
                    speed=speed
                )

            elif mode == "left_pickers":
                pos_str = self.left_picker_position_var.get()
                speed = int(self.speed_var.get())
                LEFT_MAP = {
                    'ALL_UP': 0,
                    'PICK_BLOCKS': 2,
                    'HOLD_BLOCKS': 3,
                    'ROVER': 4,
                    'DROP_BLOCKS': 1
                }
                position = LEFT_MAP.get(pos_str, 3)
                self.planner.setLeftPickersPosition(
                    position=position,
                    speed=speed
                )

            elif mode == "backarm":
                pos_str = self.ball_picker_position_var.get()
                speed = int(self.speed_var.get())
                BACK_MAP = {
                    'PICK_BALLS': 0,
                    'DROP_BALLS': 1,
                    'SLIDE_BALL': 2,
                    'PICK_BLOCKS_BACKARM': 11,
                    'HOLD_BLOCKS_BACKARM': 12,
                    'RYGW_SWAP_PICK': 13
                }
                position = BACK_MAP.get(pos_str, 0)
                self.planner.setBackarmPosition(
                    position=position,
                    speed=speed
                )
                
            elif mode == "wait":
                wait_time = int(self.wait_time_var.get())
                self.planner.wait(wait_time)
                
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid parameter value: {e}")

    def apply_right_picker_angle(self):
        """Apply a custom angle to the right/front pickers and record the command."""
        try:
            angle = float(self.right_picker_angle_var.get())
            speed = int(self.speed_var.get())
            self.planner.setRightPickersAngle(angle=angle, speed=speed)
            if hasattr(self, 'status_label'):
                self.status_label.config(text=f"Set right pickers angle to {angle:.1f}°")
            self.update_plot()
        except Exception as e:
            messagebox.showerror("Error", f"Invalid angle or speed: {e}")

    def apply_left_picker_angle(self):
        """Apply a custom angle to the left pickers and record the command."""
        try:
            angle = float(self.left_picker_angle_var.get())
            speed = int(self.speed_var.get())
            self.planner.setLeftPickersAngle(angle=angle, speed=speed)
            if hasattr(self, 'status_label'):
                self.status_label.config(text=f"Set left pickers angle to {angle:.1f}°")
            self.update_plot()
        except Exception as e:
            messagebox.showerror("Error", f"Invalid angle or speed: {e}")

    def apply_backarm_angle(self):
        """Apply a custom angle to the backarm pickers and record the command."""
        try:
            angle = float(self.backarm_angle_var.get())
            speed = int(self.speed_var.get())
            self.planner.setBackarmAngle(angle=angle, speed=speed)
            if hasattr(self, 'status_label'):
                self.status_label.config(text=f"Set backarm angle to {angle:.1f}°")
            self.update_plot()
        except Exception as e:
            messagebox.showerror("Error", f"Invalid angle or speed: {e}")
    
            
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
            # Backwards compatibility: treat as right pickers
            self._replay_setRightPickersPosition(**params)
        elif cmd['type'] == 'set_ball_picker_position':
            params = {k: v for k, v in cmd.items() if k != 'type'}
            # Backwards compatibility: treat as backarm
            self._replay_setBackarmPosition(**params)
        elif cmd['type'] == 'setRightPickersPosition':
            params = {k: v for k, v in cmd.items() if k != 'type'}
            self._replay_setRightPickersPosition(**params)
        elif cmd['type'] == 'setLeftPickersPosition':
            params = {k: v for k, v in cmd.items() if k != 'type'}
            self._replay_setLeftPickersPosition(**params)
        elif cmd['type'] == 'setBackarmPosition':
            params = {k: v for k, v in cmd.items() if k != 'type'}
            self._replay_setBackarmPosition(**params)
        
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
    
    def _replay_curve(self, radius, angle, speed=400, acceleration=450, deceleration=0, 
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
    
    def _replay_awaitarc(self, radius, angle=None, distance=None, speed=400, then="HOLD", wait=True):
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
    
    def _replay_bezier_curve(self, start, control, end, speed=400, acceleration=450):
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
    
    def _replay_set_pickers_position(self, position, wait=True, speed=600):
        """Replay set_pickers_position without adding to commands list."""
        # This doesn't change robot position, just add to commands
        cmd = {
            'type': 'set_pickers_position',
            'position': position,
            'wait': wait,
            'speed': speed
        }
        self.planner.commands.append(cmd)
    
    def _replay_set_ball_picker_position(self, position, wait=True, speed=600):
        """Replay set_ball_picker_position without adding to commands list."""
        # This doesn't change robot position, just add to commands
        cmd = {
            'type': 'set_ball_picker_position',
            'position': position,
            'wait': wait,
            'speed': speed
        }
        self.planner.commands.append(cmd)

    def _replay_setRightPickersPosition(self, position, wait=True, speed=600):
        """Replay setRightPickersPosition without adding to commands list."""
        cmd = {
            'type': 'setRightPickersPosition',
            'position': position,
            'wait': wait,
            'speed': speed
        }
        self.planner.commands.append(cmd)


    def _replay_setLeftPickersPosition(self, position, wait=True, speed=600):
        """Replay setLeftPickersPosition without adding to commands list."""
        cmd = {
            'type': 'setLeftPickersPosition',
            'position': position,
            'wait': wait,
            'speed': speed
        }
        self.planner.commands.append(cmd)


    def _replay_setBackarmPosition(self, position, wait=True, speed=600):
        """Replay setBackarmPosition without adding to commands list."""
        cmd = {
            'type': 'setBackarmPosition',
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

    def save_missions(self):
        """Save current missions list to `tools/extracted_missions.json`."""
        try:
            out = {
                'field_cm': [self.planner.field_width, self.planner.field_height],
                'missions': [ { **m, 'angle': m.get('angle', 0.0) } for m in self.missions ]
            }
            with open('tools/extracted_missions.json', 'w') as f:
                json.dump(out, f, indent=2)
            messagebox.showinfo('Success', 'Missions saved to tools/extracted_missions.json')
        except Exception as e:
            messagebox.showerror('Error', f'Failed to save missions: {e}')

    def load_missions(self):
        """Load missions from `tools/extracted_missions.json` if present and normalize entries."""
        try:
            path = 'tools/extracted_missions.json'
            if not os.path.exists(path):
                # No extracted missions - nothing to load
                return
            with open(path, 'r') as f:
                data = json.load(f)

            raw = data.get('missions', [])
            colors = ['purple', 'cyan', 'brown', 'grey', 'red', 'pink', 'lime', 'orange', 'green', 'blue', 'yellow']
            loaded = []
            for i, m in enumerate(raw):
                try:
                    name = m.get('name', f'M{i+1}')
                    x = float(m.get('x', m.get('cx', 0)))
                    y = float(m.get('y', m.get('cy', 0)))
                    w = float(m.get('w', 10))
                    h = float(m.get('h', 10))
                    angle = float(m.get('angle', 0.0))
                    color = m.get('color') or colors[i % len(colors)]
                    loaded.append({'name': name, 'x': x, 'y': y, 'w': w, 'h': h, 'angle': angle, 'color': color})
                except Exception:
                    # skip malformed entry
                    continue

            if loaded:
                self.missions = loaded
                # ensure selection index is valid
                self.selected_mission_index = None
                if hasattr(self, 'status_label'):
                    self.status_label.config(text=f"Loaded {len(self.missions)} missions")
                self.update_plot()
        except Exception as e:
            messagebox.showerror('Error', f'Failed to load missions: {e}')
            
    def generate_code(self):
        """Generate and display code."""
        if not self.planner.commands:
            messagebox.showwarning("Warning", "No path to generate code from!")
            return
            
        # Generate base code
        code = self.planner.generate_code("drawn_path")

        # Append any pasted custom code the user provided
        if getattr(self, 'pasted_code', ""):
            code += "\n\n# --- Pasted Code (imported) ---\n"
            code += self.pasted_code
        
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

    def paste_code_dialog(self):
        """Open a dialog where the user can paste a code block to append to generated code.

        If the pasted block contains a line like:
            POSITION: x,y,heading
        then the robot position will be moved to those coordinates (x,y in cm, heading in degrees).
        """
        dlg = tk.Toplevel(self.root)
        dlg.title("Paste Code Block")
        dlg.geometry("800x600")

        instructions = ttk.Label(dlg, text="Paste your code below. To move the robot after import, include a line:\nPOSITION: x,y,heading", wraplength=780)
        instructions.pack(padx=10, pady=(10, 0))

        text = tk.Text(dlg, wrap=tk.NONE, font=("Courier", 10))
        text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        btn_frame = ttk.Frame(dlg)
        btn_frame.pack(fill=tk.X, padx=10, pady=(0,10))

        def do_import():
            pasted = text.get(1.0, tk.END).rstrip()
            if not pasted:
                messagebox.showwarning("Warning", "No code pasted")
                return
            # Save pasted code
            self.pasted_code = pasted
            # Attempt to parse a POSITION comment
            moved = self._apply_position_from_text(pasted)
            # Also parse simulator function calls and import them as commands
            imported = self._parse_and_import_functions(pasted)
            if moved or imported:
                messagebox.showinfo("Imported", f"Code imported. POSITION applied: {moved}. Commands imported: {imported}.")
            else:
                messagebox.showinfo("Imported", "Code imported. No POSITION comment or recognized commands found.")
            dlg.destroy()
            self.update_plot()

        def do_cancel():
            dlg.destroy()

        ttk.Button(btn_frame, text="Import", command=do_import).pack(side=tk.RIGHT, padx=(0,5))
        ttk.Button(btn_frame, text="Cancel", command=do_cancel).pack(side=tk.RIGHT)

    def _apply_position_from_text(self, text_block: str) -> bool:
        """Parse POSITION comment and move robot if found.

        Looks for lines like:
            POSITION: 120.5, 30.0, 90
        Returns True if position was found and applied.
        """
        import re
        # Look for POSITION: x,y,heading (allow spaces)
        m = re.search(r"POSITION\s*:\s*([+-]?\d+(?:\.\d*)?)\s*,\s*([+-]?\d+(?:\.\d*)?)\s*,\s*([+-]?\d+(?:\.\d*)?)", text_block, re.IGNORECASE)
        if not m:
            # Try JSON-style position
            try:
                parsed = json.loads(text_block)
                if isinstance(parsed, dict) and 'position' in parsed:
                    pos = parsed['position']
                    if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                        x, y, heading = float(pos[0]), float(pos[1]), float(pos[2])
                        self._set_robot_position(x, y, heading)
                        return True
            except Exception:
                pass
            return False

        try:
            x = float(m.group(1))
            y = float(m.group(2))
            heading = float(m.group(3))
            self._set_robot_position(x, y, heading)
            return True
        except Exception:
            return False

    def _set_robot_position(self, x: float, y: float, heading: float):
        """Move the planner's robot to the given position and record it in history."""
        # Clamp to field bounds
        x = max(0.0, min(self.planner.field_width, x))
        y = max(0.0, min(self.planner.field_height, y))
        # Normalize heading to [-180,180]
        h = heading % 360
        if h > 180:
            h -= 360
        self.planner.x = x
        self.planner.y = y
        self.planner.heading = h
        # Append to history and set start point
        self.planner.path_history.append((x, y, h))
        self.start_point = (x, y)

    def _parse_and_import_functions(self, text_block: str) -> int:
        """Parse common simulator-generated function calls and import them as planner commands.

        Returns the number of commands imported.
        Heuristics:
        - Distances > 1000 are treated as mm and converted to cm (/10).
        - Radius arguments are treated similarly.
        """
        import re
        imported = 0

        # Helper to convert possible mm->cm
        def _to_cm(val: float) -> float:
            try:
                v = float(val)
            except Exception:
                return 0.0
            if abs(v) > 1000:  # likely mm
                return v / 10.0
            return v

        # Patterns for common function calls
        patterns = [
            # gyroStraightWithDrive(distance, speed, backward=False, target_angle=0, till_black_line=False, till_white_line=False)
            (r"gyroStraightWithDrive\s*\(([^)]*)\)", 'drive_straight'),
            # turnToAngle(angle, speed, force_turn=None)
            (r"turnToAngle\s*\(([^)]*)\)", 'turn_to_angle'),
            # curve(radius, angle, speed)
            (r"curve\s*\(([^)]*)\)", 'curve'),
            # awaitarc(radius, angle=None, distance=None, speed=..., then='HOLD', wait=True)
            (r"awaitarc\s*\(([^)]*)\)", 'awaitarc'),
            # followBlackLinePID(distance_mm, speed, edge)
            (r"followBlackLinePID\s*\(([^)]*)\)", 'follow_line'),
            # followLine(distance_mm, speed, edge)
            (r"followLine\s*\(([^)]*)\)", 'follow_line'),
        ]

        for pat, kind in patterns:
            for m in re.finditer(pat, text_block, re.IGNORECASE | re.MULTILINE):
                args = m.group(1)
                # Split arguments naively by comma but keep simple key=val
                parts = [p.strip() for p in re.split(r',(?=(?:[^\"]*\"[^\"]*\")*[^\"]*$)', args) if p.strip()]
                kwargs = {}
                positionals = []
                for p in parts:
                    if '=' in p:
                        k, v = p.split('=', 1)
                        kwargs[k.strip()] = v.strip()
                    else:
                        positionals.append(p)

                try:
                    if kind == 'drive_straight':
                        # Expect first positional = distance, second = speed
                        if positionals:
                            dist_raw = float(positionals[0])
                            distance_cm = _to_cm(dist_raw)
                        else:
                            distance_cm = _to_cm(kwargs.get('distance', 0))
                        speed = int(float(positionals[1]) if len(positionals) > 1 else float(kwargs.get('speed', 400)))
                        backward = kwargs.get('backward', 'False').lower() in ('true', '1')
                        target_angle = kwargs.get('target_angle')
                        if target_angle is not None:
                            try:
                                target_angle = float(target_angle)
                            except Exception:
                                target_angle = None
                        cmd = {
                            'type': 'drive_straight',
                            'distance': distance_cm,
                            'speed': speed,
                            'backward': backward,
                            'target_angle': target_angle,
                            'till_black_line': kwargs.get('till_black_line', 'False').lower() in ('true','1'),
                            'till_white_line': kwargs.get('till_white_line', 'False').lower() in ('true','1')
                        }
                        # Replay/import
                        self.replay_command(cmd)
                        imported += 1

                    elif kind == 'turn_to_angle':
                        # First positional = angle
                        if positionals:
                            angle = float(positionals[0])
                        else:
                            angle = float(kwargs.get('angle', 0))
                        speed = int(float(positionals[1]) if len(positionals) > 1 else float(kwargs.get('speed', 400)))
                        # Store target_angle as negative to match replay_command's expectations
                        cmd = {'type': 'turn_to_angle', 'target_angle': -angle, 'speed': speed}
                        self.replay_command(cmd)
                        imported += 1

                    elif kind == 'curve':
                        # curve(radius, angle, speed)
                        if positionals:
                            radius_raw = float(positionals[0])
                            radius_cm = _to_cm(radius_raw)
                            angle = float(positionals[1]) if len(positionals) > 1 else float(kwargs.get('angle', 90))
                        else:
                            radius_cm = _to_cm(kwargs.get('radius', 0))
                            angle = float(kwargs.get('angle', 90))
                        cmd = {'type': 'curve', 'radius': radius_cm, 'angle': angle, 'speed': int(float(kwargs.get('speed', 400)))}
                        self.replay_command(cmd)
                        imported += 1

                    elif kind == 'awaitarc':
                        # awaitarc(radius, angle=None, distance=None, speed=...)
                        # Support both positional and keyword forms
                        radius_raw = None
                        angle = None
                        distance = None
                        if positionals:
                            radius_raw = float(positionals[0])
                            if len(positionals) > 1:
                                # could be angle or distance
                                angle = float(positionals[1])
                        else:
                            radius_raw = float(kwargs.get('radius', 0))
                            if 'angle' in kwargs:
                                angle = float(kwargs.get('angle'))
                            if 'distance' in kwargs:
                                distance = float(kwargs.get('distance'))
                        radius_cm = _to_cm(radius_raw)
                        speed = int(float(kwargs.get('speed', 400)))
                        cmd = {'type': 'awaitarc', 'radius': radius_cm, 'angle': angle, 'distance': distance, 'speed': speed, 'then': kwargs.get('then','HOLD'), 'wait': kwargs.get('wait','True').lower() in ('true','1')}
                        self.replay_command(cmd)
                        imported += 1

                    elif kind == 'follow_line':
                        # followLine(distance_mm, speed, edge)
                        if positionals:
                            dist_raw = float(positionals[0])
                            # follow_line expects distance in mm in replay; keep as-is
                            distance_mm = dist_raw
                        else:
                            distance_mm = float(kwargs.get('distance', 0))
                        speed = int(float(positionals[1]) if len(positionals) > 1 else float(kwargs.get('speed', 400)))
                        edge = kwargs.get('edge', positionals[2] if len(positionals) > 2 else 'left')
                        cmd = {'type': 'follow_line', 'distance': distance_mm, 'speed': speed, 'edge': edge}
                        self.replay_command(cmd)
                        imported += 1

                except Exception:
                    # skip malformed parse
                    continue

        return imported
            
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


def main():
    """Main function to run the path drawer."""
    root = tk.Tk()
    app = PathDrawer(root)
    root.mainloop()


if __name__ == "__main__":
    main() 