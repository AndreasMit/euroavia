"""
GCS Visualiser Utility Library
Contains helper functions, WebSocket handling, and test mode functionality for the GCS Visualiser.
"""
import math
import time
import random
import json
import asyncio
import websockets
import webbrowser
import os
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Global list of connected WebSocket clients
clients = set()
# Latest telemetry data to send to new clients
latest_telemetry = {}

# =================== WebSocket Server Functions ===================
async def websocket_handler(websocket, path):
    """Handle WebSocket connections and broadcast telemetry data."""
    # Register new client
    clients.add(websocket)
    try:
        # Send latest telemetry data to the new client
        if latest_telemetry:
            await websocket.send(json.dumps(latest_telemetry))
        
        # Keep connection alive and handle client messages if needed
        async for message in websocket:
            # Process client messages if needed
            pass
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        # Unregister client on disconnection
        clients.remove(websocket)

async def broadcast_telemetry(data):
    """Send telemetry data to all connected WebSocket clients."""
    global latest_telemetry
    latest_telemetry = data
    if clients:  # Check if there are any connected clients
        # Create message tasks for each client
        disconnected_clients = set()
        for client in clients:
            try:
                await client.send(json.dumps(data))
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.add(client)
        
        # Remove disconnected clients
        for client in disconnected_clients:
            clients.remove(client)

def start_websocket_server(port):
    """Start the WebSocket server in a background thread."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    start_server = websockets.serve(
        websocket_handler, "localhost", port
    )
    loop.run_until_complete(start_server)
    print(f"WebSocket server started on ws://localhost:{port}")
    return loop

def launch_map_visualization(base_dir):
    """Open the map visualization in the default web browser."""
    map_path = os.path.join(base_dir, "map_visualiser.html")
    webbrowser.open('file://' + map_path)
    print(f"Opening map visualization: {map_path}")

# =================== Helper Functions ===================
def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate the great-circle distance between two points on Earth."""
    # Earth's radius in meters
    R_earth = 6371000
    
    # Convert latitudes and longitudes from degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Differences in coordinates
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    # Haversine formula components
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    # Distance between the two points in meters
    distance = R_earth * c
    return distance

# =================== Test Mode Classes and Functions ===================
class TestSerial:
    """Mock Serial class for test mode."""
    def __init__(self, home_coords):
        self.in_waiting = True
        self.last_data_time = time.time() - 0.5  # Initialize to generate data immediately
        self.test_cmd_received = "0"
        self.test_cmd_received_time = 0
        self.lat_home, self.lon_home = home_coords
    
    def readline(self):
        """Simulate reading a line from serial port."""
        # Generate new data approximately at the rate we would receive it
        current_time = time.time()
        if current_time - self.last_data_time >= 0.5:  # Simulate ~2Hz telemetry
            self.last_data_time = current_time
            self.in_waiting = True
            return self._generate_test_telemetry()
        # No data ready yet
        self.in_waiting = False
        return b""
    
    def write(self, data):
        """Simulate writing to serial port."""
        print(f"Test mode: Command sent: {data.decode('utf-8').strip()}")
        # 70% chance of successful command reception for testing both success and failure cases
        if random.random() < 0.7:
            self.test_cmd_received = "1"
            self.test_cmd_received_time = time.time()  # Record when command was received
            print("Test mode: Command received successfully")
        return len(data)

    def _generate_test_telemetry(self):
        """Generate simulated telemetry data for test mode."""
        # Create simulated data with some variance to look realistic
        timestamp = time.time()
        angle = round(random.uniform(0, 15), 1)
        altitude = round(random.uniform(100, 500), 1)
        g_force = round(random.uniform(0.9, 1.2), 2)
        voltage = round(random.uniform(11.1, 12.6), 2)
        current = round(random.uniform(0.5, 2.0), 2)
        
        # Simulated flight path - circular pattern around home coordinates
        t = (time.time() % 60) / 60 * 2 * math.pi  # Complete circle every 60 seconds
        radius = 0.001  # Roughly 100m at equator
        lat = self.lat_home + radius * math.sin(t)
        lon = self.lon_home + radius * math.cos(t)
        
        speed = round(random.uniform(5, 15), 1)
        
        # Keep command received flag as "1" for 3 seconds after receiving a command, with 70% probability
        if self.test_cmd_received == "1":
            if time.time() - self.test_cmd_received_time > 3 or random.random() > 0.7:
                self.test_cmd_received = "0"
                print("Test mode: Command received flag reset")
        
        # Format as CSV string similar to what would come from serial
        telemetry_str = f"{timestamp:.2f},{angle},{altitude},{g_force},{voltage},{current},{lat:.6f},{lon:.6f},{speed},{self.test_cmd_received}"
        return telemetry_str.encode('utf-8')

# =================== Plotting Functions ===================
class PlotWindow:
    """Class to manage telemetry plot windows"""
    
    def __init__(self, parent, plot_data, plotable_fields, plot_window_size, plot_counter, 
                 plot_windows_list=None):
        """Initialize a new plot window"""
        self.parent = parent
        self.plot_data = plot_data
        self.plotable_fields = plotable_fields
        self.plot_window_size = plot_window_size
        self.plot_counter = plot_counter
        self.plot_windows_list = plot_windows_list if plot_windows_list is not None else []
        
        # Create the window
        self.new_window = tk.Toplevel(parent)
        self.new_window.title(f"Real-Time Telemetry Plot {len(self.plot_windows_list) + 1}")
        self.new_window.attributes("-topmost", True)
        self.new_window.geometry("450x300")
        
        # Create variables for plot configuration
        self.field_var = tk.StringVar()
        self.fixed_y_var = tk.BooleanVar(value=False)
        self.y_min_val = tk.DoubleVar()
        self.y_max_val = tk.DoubleVar()
        
        # Setup the UI components
        self._setup_controls()
        self._setup_plot_area()
        
        # Store window info and setup close handler
        self.window_info = {
            "window": self.new_window,
            "update_func": self.update_plot,
            "field_var": self.field_var,
            "fixed_y_var": self.fixed_y_var,
            "y_min_val": self.y_min_val,
            "y_max_val": self.y_max_val
        }
        
        # Add to list of plot windows if list was provided
        if self.plot_windows_list is not None:
            self.plot_windows_list.append(self.window_info)
        
        # Handle window close event
        self.new_window.protocol("WM_DELETE_WINDOW", self._on_close)
    
    def _setup_controls(self):
        """Set up the control elements for the plot window"""
        # Create frame for controls (dropdown and + button)
        control_frame = tk.Frame(self.new_window)
        control_frame.pack(fill="x", pady=5)
        
        # Create dropdown menu for selecting telemetry field
        tk.Label(control_frame, text="Select Telemetry Field:", font=("Arial", 12)).pack(side="left", padx=5)
        field_dropdown = ttk.Combobox(control_frame, textvariable=self.field_var, values=self.plotable_fields, 
                                     state="readonly", font=("Arial", 12), width=20)
        field_dropdown.pack(side="left", padx=5)
        field_dropdown.current(0)  # Set to first field by default
        
        # Add (+) button for creating new plot windows
        add_button = tk.Button(control_frame, text="+", font=("Arial", 12, "bold"), 
                              command=self.create_new_plot_window, width=3)
        add_button.pack(side="right", padx=10)
        
        # Create a second control frame for y-axis limit controls
        y_control_frame = tk.Frame(self.new_window)
        y_control_frame.pack(fill="x", pady=0)
        
        # Add checkbox for fixed y-axis limits
        fixed_y_check = tk.Checkbutton(y_control_frame, text="Fixed Y-Axis Limits", 
                                      variable=self.fixed_y_var, command=self._on_fixed_y_changed,
                                      font=("Arial", 10))
        fixed_y_check.pack(side="left", padx=5)
        
        # Bind dropdown selection change event
        field_dropdown.bind("<<ComboboxSelected>>", self._on_field_change)
    
    def _setup_plot_area(self):
        """Set up the matplotlib plot area"""
        # Create matplotlib figure for plotting
        self.fig = Figure(figsize=(8, 6), dpi=75)
        self.plot_area = self.fig.add_subplot(111)
        self.plot_area.set_title(f"{self.plotable_fields[0]} over Time")
        self.plot_area.set_ylabel(self.plotable_fields[0])
        self.plot_area.grid(True)
        
        # Create canvas for the plot
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.new_window)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)
    
    def update_plot(self):
        """Update the plot with current data"""
        field = self.field_var.get()
        if field and field in self.plot_data and self.plot_data[field]['x']:
            self.plot_area.clear()
            self.plot_area.plot(self.plot_data[field]['x'][-self.plot_window_size:], 
                               self.plot_data[field]['y'][-self.plot_window_size:], 'b.-')
            self.plot_area.set_title(f"{field} over Time")
            self.plot_area.set_ylabel(field)
            self.plot_area.grid(True)
            
            # Handle y-axis limits
            if self.fixed_y_var.get() and self.y_min_val.get() < self.y_max_val.get():
                # Use custom limits
                self.plot_area.set_ylim([self.y_min_val.get(), self.y_max_val.get()])
            elif len(self.plot_data[field]['y'][-self.plot_window_size:]) > 1:
                # Auto-scale with padding
                y_values = self.plot_data[field]['y'][-self.plot_window_size:]
                y_min, y_max = min(y_values), max(y_values)
                padding = (y_max - y_min) * 0.1 if y_max != y_min else 0.5
                self.plot_area.set_ylim([y_min - padding, y_max + padding])
        
        self.canvas.draw()
    
    def _on_field_change(self, event):
        """Handle field change in dropdown menu"""
        if self.fixed_y_var.get():
            # Ask if user wants to set new limits for the new field
            set_y_limits_dialog(self.new_window, self.field_var.get(), self.y_min_val, self.y_max_val)
        self.update_plot()
    
    def _on_fixed_y_changed(self):
        """Handle change in fixed y-axis checkbox"""
        if self.fixed_y_var.get():
            # Show dialog for entering y-axis limits
            set_y_limits_dialog(self.new_window, self.field_var.get(), self.y_min_val, self.y_max_val)
        else:
            # Reset to auto-scaling
            self.y_min_val.set(0)
            self.y_max_val.set(0)
            self.update_plot()
    
    def _on_close(self):
        """Handle window close event"""
        if self.plot_windows_list is not None and self.window_info in self.plot_windows_list:
            self.plot_windows_list.remove(self.window_info)
        self.new_window.destroy()
    
    def create_new_plot_window(self):
        """Create a new plot window"""
        return PlotWindow(self.parent, self.plot_data, self.plotable_fields, 
                         self.plot_window_size, self.plot_counter, self.plot_windows_list)

def set_y_limits_dialog(parent, field_name, y_min_var, y_max_var):
    """Creates a dialog for setting y-axis limits"""
    dialog = tk.Toplevel(parent)
    dialog.title(f"Set Y-Axis Limits for {field_name}")
    dialog.geometry("300x150")
    dialog.transient(parent)  # Make it a child of parent
    dialog.grab_set()  # Make it modal
    
    # Get current values or default values
    current_min = y_min_var.get() if y_min_var.get() != 0 else ""
    current_max = y_max_var.get() if y_max_var.get() != 0 else ""
    
    # Create widgets
    tk.Label(dialog, text=f"Set Y-Axis Limits for {field_name}", font=("Arial", 12)).pack(pady=5)
    
    limits_frame = tk.Frame(dialog)
    limits_frame.pack(pady=5)
    
    tk.Label(limits_frame, text="Min Value:", font=("Arial", 10)).grid(row=0, column=0, padx=5, pady=5, sticky="e")
    min_entry = tk.Entry(limits_frame, font=("Arial", 10), width=10)
    min_entry.grid(row=0, column=1, padx=5, pady=5)
    if current_min != "":
        min_entry.insert(0, str(current_min))
    
    tk.Label(limits_frame, text="Max Value:", font=("Arial", 10)).grid(row=1, column=0, padx=5, pady=5, sticky="e")
    max_entry = tk.Entry(limits_frame, font=("Arial", 10), width=10)
    max_entry.grid(row=1, column=1, padx=5, pady=5)
    if current_max != "":
        max_entry.insert(0, str(current_max))
    
    def on_ok():
        # Validate input
        try:
            min_val = float(min_entry.get())
            max_val = float(max_entry.get())
            
            if min_val >= max_val:
                tk.messagebox.showerror("Invalid Input", "Min value must be less than max value")
                return
            
            # Set variables
            y_min_var.set(min_val)
            y_max_var.set(max_val)
            dialog.destroy()
        except ValueError:
            tk.messagebox.showerror("Invalid Input", "Please enter valid numeric values")
    
    def on_cancel():
        # Reset checkbox and destroy dialog
        for window_info in parent.winfo_children():
            if hasattr(window_info, 'fixed_y_var') and window_info.fixed_y_var == y_min_var:
                window_info.fixed_y_var.set(False)
        dialog.destroy()
    
    # Buttons
    button_frame = tk.Frame(dialog)
    button_frame.pack(pady=10)
    
    tk.Button(button_frame, text="OK", command=on_ok, width=10).grid(row=0, column=0, padx=5)
    tk.Button(button_frame, text="Cancel", command=on_cancel, width=10).grid(row=0, column=1, padx=5)
    
    # Center the dialog relative to parent
    dialog.update_idletasks()
    width = dialog.winfo_width()
    height = dialog.winfo_height()
    x = parent.winfo_rootx() + (parent.winfo_width() // 2) - (width // 2)
    y = parent.winfo_rooty() + (parent.winfo_height() // 2) - (height // 2)
    dialog.geometry(f"{width}x{height}+{x}+{y}")
    
    # Make sure the dialog stays on top
    dialog.focus_set()
    dialog.wait_window()

def update_all_plots(plot_windows):
    """Updates all plot windows with latest data"""
    for window_info in plot_windows:
        try:
            if window_info["window"].winfo_exists():
                window_info["update_func"]()
        except Exception as e:
            print(f"Error updating plot: {e}")
