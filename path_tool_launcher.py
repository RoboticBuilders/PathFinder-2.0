# -*- coding: utf-8 -*-
"""
Path Tool Launcher for LEGO SPIKE Prime Robot
Choose between Path Planner and Path Drawer modes.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import sys
import os

class PathToolLauncher:
    """Launcher for path planning tools."""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Path Tool Launcher - LEGO SPIKE Prime Robot")
        self.root.geometry("550x450")
        self.root.resizable(False, False)
        
        # Center the window
        self.center_window()
        
        self.setup_ui()
        
    def center_window(self):
        """Center the window on screen."""
        self.root.update_idletasks()
        width = self.root.winfo_width()
        height = self.root.winfo_height()
        x = (self.root.winfo_screenwidth() // 2) - (width // 2)
        y = (self.root.winfo_screenheight() // 2) - (height // 2)
        self.root.geometry(f'{width}x{height}+{x}+{y}')
        
    def setup_ui(self):
        """Setup the user interface."""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="25")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title_label = ttk.Label(main_frame, text="Path Planning Tools", 
                               font=("Arial", 18, "bold"))
        title_label.pack(pady=(0, 30))
        
        # Subtitle
        subtitle_label = ttk.Label(main_frame, text="Choose your preferred path planning method:", 
                                  font=("Arial", 12))
        subtitle_label.pack(pady=(0, 20))
        
        # Mode selection frame
        mode_frame = ttk.Frame(main_frame)
        mode_frame.pack(fill=tk.X, pady=(0, 30))
        
        # Path Planner Option
        planner_frame = ttk.LabelFrame(mode_frame, text="Path Planner", padding="20")
        planner_frame.pack(fill=tk.X, pady=(0, 20))
        
        planner_desc = ttk.Label(planner_frame, text="Programmatic path creation using Python code.\n"
                                "Best for precise control and complex logic.", 
                                font=("Arial", 10))
        planner_desc.pack(pady=(0, 10))
        
        planner_btn = ttk.Button(planner_frame, text="Launch Path Planner", 
                                command=self.launch_planner)
        planner_btn.pack(fill=tk.X, pady=(0, 5))
        
        # Path Drawer Option
        drawer_frame = ttk.LabelFrame(mode_frame, text="Path Drawer", padding="20")
        drawer_frame.pack(fill=tk.X)
        
        drawer_desc = ttk.Label(drawer_frame, text="Visual path drawing with mouse interactions.\n"
                               "Best for quick prototyping and visual planning.", 
                               font=("Arial", 10))
        drawer_desc.pack(pady=(0, 10))
        
        drawer_btn = ttk.Button(drawer_frame, text="Launch Path Drawer", 
                               command=self.launch_drawer)
        drawer_btn.pack(fill=tk.X, pady=(0, 5))
        
        # Separator
        separator = ttk.Separator(main_frame, orient='horizontal')
        separator.pack(fill=tk.X, pady=20)
        
        # Quick actions frame
        actions_frame = ttk.LabelFrame(main_frame, text="Quick Actions", padding="20")
        actions_frame.pack(fill=tk.X)
        
        # Example path button
        example_btn = ttk.Button(actions_frame, text="Create Example Path", 
                                command=self.create_example)
        example_btn.pack(fill=tk.X, pady=(0, 5))
        
        # Documentation button
        docs_btn = ttk.Button(actions_frame, text="View Documentation", 
                             command=self.show_documentation)
        docs_btn.pack(fill=tk.X, pady=(0, 5))
        
        # About button
        about_btn = ttk.Button(actions_frame, text="About", 
                              command=self.show_about)
        about_btn.pack(fill=tk.X)
        
        # Status bar
        self.status_var = tk.StringVar(value="Ready to launch path planning tools")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, 
                              relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(fill=tk.X, pady=(20, 0))
        
    def launch_planner(self):
        """Launch the path planner."""
        try:
            self.status_var.set("Launching Path Planner...")
            self.root.update()
            
            # Import and run the planner
            from path_planner import create_example_path
            
            # Create example and show visualization
            planner = create_example_path()
            planner.visualize_path()
            
            # Generate and show code
            code = planner.generate_code("example_path")
            self.show_code_window("Example Path Code", code)
            
            self.status_var.set("Path Planner launched successfully")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to launch Path Planner: {e}")
            self.status_var.set("Error launching Path Planner")
            
    def launch_drawer(self):
        """Launch the path drawer."""
        try:
            self.status_var.set("Launching Path Drawer...")
            self.root.update()
            
            # Launch the drawer in a new process
            subprocess.Popen([sys.executable, "path_drawer.py"])
            
            self.status_var.set("Path Drawer launched successfully")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to launch Path Drawer: {e}")
            self.status_var.set("Error launching Path Drawer")
            
    def create_example(self):
        """Create and show an example path."""
        try:
            self.status_var.set("Creating example path...")
            self.root.update()
            
            from path_planner import create_example_path
            
            planner = create_example_path()
            
            # Show visualization
            planner.visualize_path()
            
            # Show code
            code = planner.generate_code("example_path")
            self.show_code_window("Example Path Code", code)
            
            self.status_var.set("Example path created successfully")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to create example: {e}")
            self.status_var.set("Error creating example")
            
    def show_code_window(self, title, code):
        """Show code in a new window."""
        code_window = tk.Toplevel(self.root)
        code_window.title(title)
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
        
    def show_documentation(self):
        """Show documentation."""
        doc_text = """
Path Planning Tools Documentation

PATH PLANNER:
- Use Python code to create precise robot paths
- Supports all movement commands from utils.py
- Generates accurate distance and angle calculations
- Best for complex logic and precise control

PATH DRAWER:
- Draw paths visually using mouse interactions
- Automatic code generation from drawings
- Real-time path visualization
- Best for quick prototyping and visual planning

SUPPORTED COMMANDS:
- drive_straight(): Straight line movement
- turn_to_angle(): Turning to specific angles
- curve(): Curved movements with radius
- follow_line(): Line following with PID
- set_pickers_position(): Arm positioning
- set_ball_picker_position(): Ball picker positioning
- wait(): Timing delays

USAGE:
1. Choose your preferred tool
2. Create your path
3. Generate Python code
4. Copy code to your robot program
        """
        
        doc_window = tk.Toplevel(self.root)
        doc_window.title("Documentation")
        doc_window.geometry("600x500")
        
        text_widget = tk.Text(doc_window, wrap=tk.WORD, font=("Arial", 10))
        text_widget.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        scrollbar = ttk.Scrollbar(doc_window, orient=tk.VERTICAL, command=text_widget.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        text_widget.configure(yscrollcommand=scrollbar.set)
        
        text_widget.insert(tk.END, doc_text)
        text_widget.config(state=tk.DISABLED)
        
    def show_about(self):
        """Show about information."""
        about_text = """
Path Planning Tools for LEGO SPIKE Prime Robot

Version: 1.0
Author: AI Assistant

This tool provides two methods for planning robot paths:

1. Path Planner: Programmatic approach using Python code
2. Path Drawer: Visual approach using mouse interactions

Both tools generate Python code compatible with your utils.py functions
and provide accurate distance and angle calculations for robot movements.

Features:
- Real-time path visualization
- Automatic code generation
- Support for all movement commands
- Path saving and loading
- Example path creation
        """
        
        messagebox.showinfo("About", about_text)


def main():
    """Main function to run the launcher."""
    root = tk.Tk()
    app = PathToolLauncher(root)
    root.mainloop()


if __name__ == "__main__":
    main() 