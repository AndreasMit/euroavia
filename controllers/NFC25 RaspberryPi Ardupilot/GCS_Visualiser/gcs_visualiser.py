# =================== Imports and Parameter Configuration ===================
import tkinter as tk
import serial
import time
import csv                     # For CSV writing
from datetime import datetime  # For timestamped filenames
import math                    # For distance calculation

# -------------------- Configuration Parameters --------------------
SERIAL_PORT = 'COM12'         # Serial port to connect to the GCS device
BAUD_RATE = 9600              # Baud rate for serial communication
GUI_REFRESH_RATE = 100        # GUI update interval in milliseconds
SAVE_TO_FILE = False          # Set to True to enable CSV logging

# ----- Command Sender Configuration (customizable) -----
COMMAND_SEND_DELAY = 0.5      # Delay in seconds between command sends
COMMAND_NUMBER_SEND = 7       # Total number of times to send a command

# ----- Pattern Definitions for Commands -----
START_PATTERN = "/*"
END_PATTERN = "*/"

# ----- Ground Distance Calculation Configuration -----
lat_home, lon_home = 37.9780196, 23.783700  # Home coordinates for distance calculation

# ----- Button Colors -----
BUTTON_DEFAULT_COLOR = "SystemButtonFace"  # Default button color
BUTTON_WAITING_COLOR = "yellow"            # When command is sent but not confirmed
BUTTON_SUCCESS_COLOR = "green"             # When command is confirmed (received flag = 1)
BUTTON_FAILED_COLOR = "red"                # When command times out without confirmation

# =================== Helper Functions ===================
# Haversine distance calculation function from test.py
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

# =================== CSV and Serial Initialization ===================
# Initialize CSV file for logging if enabled
if SAVE_TO_FILE:
    start_time = datetime.now()
    filename = "GCS_Visualiser/Incoming_Telemetry_CSV" + start_time.strftime("%Y%m%d_%H%M%S") + ".csv"
    csv_file = open(filename, "w", newline="")
    csv_writer = csv.writer(csv_file)
    # Write header row to CSV
    csv_writer.writerow(["Timestamp", "Latency (ms)", "Freq (Hz)", "Angle of Attack", "Altitude", 
                           "G-Force", "Battery Voltage", "Battery Current", "Latitude", "Longitude", "Speed", "Cmd Received"])

# Initialize serial connection to read telemetry data
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Global flags and variables for telemetry handling
telemetry_paused = False  # Pauses telemetry reading during command send
prev_msg_timestamp = None

# Global variables for command status tracking
command_sent_time = 0
waiting_for_confirmation = False
success_time = 0
button_reset_after = None

# =================== Main Window Creation ===================
# Create and configure the main telemetry display window
root = tk.Tk()
root.title("GCS Telemetry Visualiser")
root.attributes("-topmost", True)  # Ensure main window stays on top

# -------------------- GUI Labels Setup --------------------
# Update labels to include Ground Distance
labels_text = ["Timestamp", "Latency (ms)", "Freq (Hz)", "Angle of Attack", "Altitude", 
               "G-Force", "Battery Voltage", "Battery Current", "Latitude", "Longitude", 
               "Speed", "Ground Distance", "Cmd Received"]
labels = {}
for i, text in enumerate(labels_text):
    tk.Label(root, text=text+":", font=("Arial", 12)).grid(row=i, column=0, padx=5, pady=2, sticky="e")
    labels[text] = tk.Label(root, text="N/A", font=("Arial", 12))
    labels[text].grid(row=i, column=1, padx=5, pady=2, sticky="w")

# =================== Telemetry Update Function ===================
def update_gui():
    """Reads telemetry from serial port and updates GUI labels."""
    global prev_msg_timestamp, waiting_for_confirmation, success_time, button_reset_after
    
    # Check for button color reset after success
    if button_reset_after is not None and time.time() >= button_reset_after:
        send_button.config(bg=BUTTON_DEFAULT_COLOR)
        button_reset_after = None
    
    # Check for command timeout (5 seconds without confirmation)
    if waiting_for_confirmation and time.time() - command_sent_time > 5:
        send_button.config(bg=BUTTON_FAILED_COLOR)
        waiting_for_confirmation = False
    
    if telemetry_paused:
        root.after(GUI_REFRESH_RATE, update_gui)
        return
    try:
        if ser.in_waiting:
            raw_line = ser.readline()
            line = raw_line.decode('utf-8', errors='replace').strip()
            parts = line.split(',')
            # Expecting 10 fields:
            # 0: Timestamp, 1: Angle, 2: Altitude, 3: G-Force, 4: Bat Voltage, 5: Bat Current,
            # 6: Latitude, 7: Longitude, 8: Speed, 9: Cmd Received
            if len(parts) == 10:
                current_msg_timestamp = float(parts[0])
                latency = (time.time() - current_msg_timestamp) * 1000  
                if prev_msg_timestamp is not None:
                    elapsed = current_msg_timestamp - prev_msg_timestamp
                    freq = 1 / elapsed if elapsed > 0 else 0
                else:
                    freq = 0
                prev_msg_timestamp = current_msg_timestamp
                
                # Calculate ground distance if we have valid coordinates
                ground_distance = "N/A"
                try:
                    lat = float(parts[6])
                    lon = float(parts[7])
                    if lat != 0 and lon != 0:  # Make sure we have real coordinates
                        distance = haversine_distance(lat_home, lon_home, lat, lon)
                        ground_distance = f"{distance:.1f} m"
                except (ValueError, IndexError):
                    print(f"Error calculating distance: {parts[6]}, {parts[7]} \t Error: {ValueError} - {IndexError}")
                
                # Process command received flag
                if waiting_for_confirmation and parts[9] == "1":
                    # Command was received successfully
                    waiting_for_confirmation = False
                    success_time = time.time()
                    send_button.config(bg=BUTTON_SUCCESS_COLOR)
                    button_reset_after = time.time() + 2  # Reset to default after 2 seconds
                
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
                    csv_writer.writerow([data_map[key] for key in labels_text])
                    csv_file.flush()
    except Exception as e:
        print("Error reading serial:", e)
    root.after(GUI_REFRESH_RATE, update_gui)  # Schedule the next update

# =================== Command Sender Functions ===================
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

def send_command():
    """Initiates the command send sequence by extracting command text."""
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
# Create a secondary window for entering and sending commands
cmd_window = tk.Toplevel(root)
cmd_window.title("Command Sender")
cmd_window.attributes("-topmost", True)  # Ensure the command sender window remains above others
tk.Label(cmd_window, text="Enter Command:", font=("Arial", 12)).grid(row=0, column=0, padx=5, pady=5)
command_entry = tk.Entry(cmd_window, font=("Arial", 12), width=40)
command_entry.grid(row=0, column=1, padx=5, pady=5)
send_button = tk.Button(cmd_window, text="SEND", font=("Arial", 12), command=send_command)
send_button.grid(row=1, column=0, columnspan=2, padx=5, pady=5)
# Bind the Enter key to trigger send_command() for convenience
command_entry.bind("<Return>", lambda event: send_command())

# =================== Application Main Loop ===================
# Start the telemetry update loop and run the GUI
update_gui()
root.mainloop()
