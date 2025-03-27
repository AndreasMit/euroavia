# =================== Imports and Parameter Configuration ===================
import tkinter as tk
import serial
import time
import csv                     # For CSV writing
from datetime import datetime  # For timestamped filenames
import math                    # For distance calculation
import random                  # For test mode data generation

# -------------------- Configuration Parameters --------------------
SERIAL_PORT = 'COM12'         # Serial port to connect to the GCS device
SERIAL_PORT = 'TEST'        # Uncomment to activate test mode
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

# ----- Test Mode Configuration -----
TEST_MODE = SERIAL_PORT == 'TEST'  # Flag to determine if we're in test mode
last_test_data_time = time.time()  # Track time of last test data generation
test_cmd_received = "0"           # Command received flag for test mode
test_cmd_received_time = 0        # Time when command was last received

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

# =================== Test Mode Functions ===================
def generate_test_telemetry():
    """Generate simulated telemetry data for test mode."""
    global test_cmd_received, last_test_data_time, test_cmd_received_time
    
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
    lat = lat_home + radius * math.sin(t)
    lon = lon_home + radius * math.cos(t)
    
    speed = round(random.uniform(5, 15), 1)
    
    # Keep command received flag as "1" for 3 seconds after receiving a command, with 70% probability
    if test_cmd_received == "1":
        if time.time() - test_cmd_received_time > 3 or random.random() > 0.7:
            test_cmd_received = "0"
            print("Test mode: Command received flag reset")
    
    last_test_data_time = time.time()
    
    # Format as CSV string similar to what would come from serial
    telemetry_str = f"{timestamp:.2f},{angle},{altitude},{g_force},{voltage},{current},{lat:.6f},{lon:.6f},{speed},{test_cmd_received}"
    return telemetry_str.encode('utf-8')

class TestSerial:
    """Mock Serial class for test mode."""
    def __init__(self):
        self.in_waiting = True
        self.last_data_time = time.time() - 0.5  # Initialize to generate data immediately
    
    def readline(self):
        """Simulate reading a line from serial port."""
        # Generate new data approximately at the rate we would receive it
        current_time = time.time()
        if current_time - self.last_data_time >= 0.5:  # Simulate ~2Hz telemetry
            self.last_data_time = current_time
            self.in_waiting = True
            return generate_test_telemetry()
        # No data ready yet
        self.in_waiting = False
        return b""
    
    def write(self, data):
        """Simulate writing to serial port."""
        global test_cmd_received, test_cmd_received_time
        print(f"Test mode: Command sent: {data.decode('utf-8').strip()}")
        # 70% chance of successful command reception for testing both success and failure cases
        if random.random() < 0.7:
            test_cmd_received = "1"
            test_cmd_received_time = time.time()  # Record when command was received
            print("Test mode: Command received successfully")
        return len(data)

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
if TEST_MODE:
    print("\n\nRUNNING IN TEST MODE - Simulating telemetry data")
    ser = TestSerial()
else:
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
        # In test mode, we always check for data since TestSerial.in_waiting is managed differently
        if TEST_MODE or ser.in_waiting:
            raw_line = ser.readline()
            # Skip empty lines (happens in test mode when no new data is ready)
            if not raw_line:
                root.after(GUI_REFRESH_RATE, update_gui)
                return
                
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
                
                # Process command received flag - fixing the issue where test_cmd_received never shows as "1" in the UI
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
    
    # Force update more frequently in test mode to catch command confirmation
    if TEST_MODE:
        root.after(50, update_gui)  # Update more frequently in test mode
    else:
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
        
        # This ensures the last command sent will be properly detected in test mode
        if TEST_MODE:
            # Force a refresh of the GUI to immediately show command status
            update_gui()

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
