# =================== Imports and Parameter Configuration ===================
import tkinter as tk
import serial
import time
import sys
import signal
import csv                     # For CSV writing
from datetime import datetime  # For timestamped filenames
import asyncio                 # For async WebSocket server
import threading               # For running WebSocket server in a thread
import os                      # For file path handling

# Import utility functions from gcs_utils
from gcs_utils import (haversine_distance, TestSerial, start_websocket_server, 
                      broadcast_telemetry, launch_map_visualization)

# -------------------- Configuration Parameters --------------------
SERIAL_PORT = 'TEST'          # Use 'TEST' for test mode or 'COM12' for real device
BAUD_RATE = 9600              # Baud rate for serial communication
GUI_REFRESH_RATE = 100        # GUI update interval in milliseconds
SAVE_TO_FILE = False          # Set to True to enable CSV logging
WEBSOCKET_PORT = 8765         # Port for WebSocket server
ENABLE_MAP = True             # Enable map visualization

# ----- Command Sender Configuration (customizable) -----
COMMAND_SEND_DELAY = 0.5      # Delay in seconds between command sends
COMMAND_NUMBER_SEND = 7       # Total number of times to send a command
START_PATTERN = "/*"          # Command start pattern
END_PATTERN = "*/"           # Command end pattern

# ----- Ground Distance Calculation Configuration -----
lat_home, lon_home = 37.977864, 23.783953  # Home coordinates for distance calculation

# ----- Button Colors -----
BUTTON_DEFAULT_COLOR = "SystemButtonFace"
BUTTON_WAITING_COLOR = "yellow"
BUTTON_SUCCESS_COLOR = "green"
BUTTON_FAILED_COLOR = "red"

# ----- Test Mode and Connection Status Configuration -----
TEST_MODE = SERIAL_PORT == 'TEST'
TELEMETRY_TIMEOUT = 2
last_telemetry_time = 0

# Global variables
websocket_loop = None         # Stores the asyncio event loop for WebSocket communications
telemetry_paused = False      # Flag to pause telemetry processing during command sending
prev_msg_timestamp = None     # Stores previous message timestamp for frequency calculation
command_sent_time = 0         # Timestamp when the last command was sent
waiting_for_confirmation = False  # Flag indicating we're waiting for command confirmation
success_time = 0              # Timestamp when command confirmation was received
button_reset_after = None     # Time when to reset the send button color to default

# =================== CSV and Serial Initialization ===================
if SAVE_TO_FILE:
    start_time = datetime.now()
    filename = "GCS_Visualiser/Incoming_Telemetry_CSV/" + start_time.strftime("%Y%m%d_%H%M%S") + ".csv"
    csv_file = open(filename, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Timestamp", "Latency (ms)", "Freq (Hz)", "Angle of Attack", "Altitude", 
                          "G-Force", "Battery Voltage", "Battery Current", "Latitude", "Longitude", "Speed", "Cmd Received"])

# Initialize serial connection
if TEST_MODE:
    print("\n\nRUNNING IN TEST MODE - Simulating telemetry data")
    ser = TestSerial(home_coords=(lat_home, lon_home))
else:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# =================== Main Window Creation ===================
root = tk.Tk()
root.title("GCS Telemetry Visualiser")
root.attributes("-topmost", True)

# -------------------- GUI Labels Setup --------------------
labels_text = ["Timestamp", "Latency (ms)", "Freq (Hz)", "Angle of Attack", "Altitude", 
               "G-Force", "Battery Voltage", "Battery Current", "Latitude", "Longitude", 
               "Speed", "Ground Distance", "Cmd Received", "Connected"]
labels = {}

# Create and place labels
for i, text in enumerate(labels_text[:-1]):
    tk.Label(root, text=text+":", font=("Arial", 12)).grid(row=i, column=0, padx=5, pady=2, sticky="e")
    labels[text] = tk.Label(root, text="N/A", font=("Arial", 12))
    labels[text].grid(row=i, column=1, padx=5, pady=2, sticky="w")

# Create LED indicator for connection status
connected_row = len(labels_text) - 1
tk.Label(root, text=labels_text[connected_row]+":", font=("Arial", 12)).grid(row=connected_row, column=0, padx=5, pady=2, sticky="e")
led_size = 15
led_canvas = tk.Canvas(root, width=led_size+8, height=led_size+8, bd=0, highlightthickness=0, bg="white")
led_canvas.grid(row=connected_row, column=1, padx=5, pady=2, sticky="w")
led_canvas.create_oval(1, 1, led_size+7, led_size+7, fill="silver", outline="#999999")
led = led_canvas.create_oval(4, 4, led_size+4, led_size+4, fill="red", outline="black", width=1)
led_canvas.create_arc(6, 6, led_size, led_size, start=40, extent=120, fill="white", outline="")

def update_led_status():
    if time.time() - last_telemetry_time < TELEMETRY_TIMEOUT:
        led_canvas.itemconfig(led, fill="green")
    else:
        led_canvas.itemconfig(led, fill="red")

# Add map visualization button
if ENABLE_MAP:
    map_button = tk.Button(root, text="Open Map Visualization", font=("Arial", 12), 
                          command=lambda: launch_map_visualization(os.path.dirname(os.path.abspath(__file__))))
    map_button.grid(row=len(labels_text), column=0, columnspan=2, padx=5, pady=10)

# =================== GUI Update and Command Functions ===================
def update_gui():
    global prev_msg_timestamp, waiting_for_confirmation, success_time, button_reset_after
    global last_telemetry_time
    
    update_led_status()
    
    if button_reset_after is not None and time.time() >= button_reset_after:
        send_button.config(bg=BUTTON_DEFAULT_COLOR)
        button_reset_after = None
    
    if waiting_for_confirmation and time.time() - command_sent_time > 5:
        send_button.config(bg=BUTTON_FAILED_COLOR)
        waiting_for_confirmation = False
    
    if telemetry_paused:
        root.after(GUI_REFRESH_RATE, update_gui)
        return
    try:
        if TEST_MODE or ser.in_waiting:
            raw_line = ser.readline()
            if not raw_line:
                root.after(GUI_REFRESH_RATE, update_gui)
                return
                
            line = raw_line.decode('utf-8', errors='replace').strip()
            parts = line.split(',')
            if len(parts) == 10:
                last_telemetry_time = time.time()
                
                current_msg_timestamp = float(parts[0])
                latency = (time.time() - current_msg_timestamp) * 1000  
                if prev_msg_timestamp is not None:
                    elapsed = current_msg_timestamp - prev_msg_timestamp
                    freq = 1 / elapsed if elapsed > 0 else 0
                else:
                    freq = 0
                prev_msg_timestamp = current_msg_timestamp
                
                ground_distance = "N/A"
                try:
                    lat = float(parts[6])
                    lon = float(parts[7])
                    if lat != 0 and lon != 0:
                        distance = haversine_distance(lat_home, lon_home, lat, lon)
                        ground_distance = f"{distance:.1f} m"
                except (ValueError, IndexError):
                    print(f"Error calculating distance: {parts[6]}, {parts[7]} \t Error: {ValueError} - {IndexError}")
                
                if waiting_for_confirmation and parts[9] == "1":
                    waiting_for_confirmation = False
                    success_time = time.time()
                    send_button.config(bg=BUTTON_SUCCESS_COLOR)
                    button_reset_after = time.time() + 2
                
                data_map = {
                    "Timestamp": parts[0],
                    "Latency (ms)": f"{latency:.0f}",
                    "Freq (Hz)": f"{freq:.1f}",
                    "Angle of Attack": parts[1],
                    "Altitude": parts[2],
                    "G-Force": parts[3],
                    "Battery Voltage": parts[4],
                    "Battery Current": parts[5],
                    "Latitude": parts[6],
                    "Longitude": parts[7],
                    "Speed": parts[8],
                    "Ground Distance": ground_distance,
                    "Cmd Received": parts[9]
                }
                for key, val in data_map.items():
                    labels[key].config(text=val)
                if SAVE_TO_FILE:
                    csv_writer.writerow([data_map[key] for key in labels_text[:-1]])
                    csv_file.flush()
                
                if ENABLE_MAP and websocket_loop:
                    websocket_data = {
                        "timestamp": float(parts[0]),
                        "altitude": float(parts[2]),
                        "lat": float(parts[6]),
                        "lon": float(parts[7]),
                        "speed": float(parts[8]),
                        "home_lat": lat_home,
                        "home_lon": lon_home,
                        "distance": ground_distance.replace(" m", "")
                    }
                    
                    asyncio.run_coroutine_threadsafe(
                        broadcast_telemetry(websocket_data), 
                        websocket_loop
                    )
    except Exception as e:
        print("Error reading serial:", e)
    
    if TEST_MODE:
        root.after(50, update_gui)
    else:
        root.after(GUI_REFRESH_RATE, update_gui)

def repeat_send(i, command):
    """Recursively sends the command at intervals defined by COMMAND_SEND_DELAY."""
    global telemetry_paused
    if i < COMMAND_NUMBER_SEND:
        # Use configured start and end patterns when building the message
        message = f"{START_PATTERN}{command}{END_PATTERN}\n"
        try:
            ser.write(message.encode('utf-8'))
            print(f"Sent command: {message.strip()}")
        except Exception as e:
            print("Error sending command:", e)
        # Schedule next send after COMMAND_SEND_DELAY seconds (converted to milliseconds)
        root.after(int(COMMAND_SEND_DELAY * 1000), lambda: repeat_send(i+1, command))
    else:
        telemetry_paused = False  # Resume telemetry after sending finished
        
        # This ensures the last command sent will be properly detected in test mode
        if TEST_MODE:
            update_gui()

def send_command():
    global telemetry_paused, command_sent_time, waiting_for_confirmation
    command = command_entry.get().strip()
    if command:
        telemetry_paused = True  # Pause telemetry reading during command send
        
        # Update command tracking variables
        command_sent_time = time.time()
        waiting_for_confirmation = True
        
        # Set button to yellow to indicate waiting for confirmation
        send_button.config(bg=BUTTON_WAITING_COLOR)
        
        # Start sending the command
        repeat_send(0, command)

# =================== Command Sender Window ===================
cmd_window = tk.Toplevel(root)
cmd_window.title("Command Sender")
cmd_window.attributes("-topmost", True)
tk.Label(cmd_window, text="Enter Command:", font=("Arial", 12)).grid(row=0, column=0, padx=5, pady=5)
command_entry = tk.Entry(cmd_window, font=("Arial", 12), width=40)
command_entry.grid(row=0, column=1, padx=5, pady=5)
send_button = tk.Button(cmd_window, text="SEND", font=("Arial", 12), command=send_command)
send_button.grid(row=1, column=0, columnspan=2, padx=5, pady=5)
command_entry.bind("<Return>", lambda event: send_command())

# =================== Start WebSocket Server ===================
if ENABLE_MAP:
    # Create a thread for the WebSocket server
    def run_websocket_server():
        global websocket_loop
        websocket_loop = start_websocket_server(WEBSOCKET_PORT)
        websocket_loop.run_forever()

    websocket_thread = threading.Thread(target=run_websocket_server, daemon=True)
    websocket_thread.start()

# =================== Application Main Loop ===================
def on_closing():
    if SAVE_TO_FILE and 'csv_file' in globals():
        print("Closing CSV file...")
        csv_file.close()
    root.destroy()

# Handle keyboard interrupts
def handle_interrupt(sig, frame):
    print("\nProgram interrupted by user. Cleaning up...")
    on_closing()
    sys.exit(0)

# Register signal handlers for clean exit
signal.signal(signal.SIGINT, handle_interrupt)
root.protocol("WM_DELETE_WINDOW", on_closing)
update_gui()
root.mainloop()

if SAVE_TO_FILE and 'csv_file' in globals():
    csv_file.close()
