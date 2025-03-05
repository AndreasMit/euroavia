# =================== Imports and Parameter Configuration ===================
import tkinter as tk
import serial
import time
import csv                     # For CSV writing
from datetime import datetime  # For timestamped filenames

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

# =================== CSV and Serial Initialization ===================
# Initialize CSV file for logging if enabled
if SAVE_TO_FILE:
    start_time = datetime.now()
    filename = "GCS_Visualiser/Incoming_Telemetry_CSV" + start_time.strftime("%Y%m%d_%H%M%S") + ".csv"
    csv_file = open(filename, "w", newline="")
    csv_writer = csv.writer(csv_file)
    # Write header row to CSV
    csv_writer.writerow(["Timestamp", "Latency (ms)", "Freq (Hz)", "Angle of Attack", "Altitude", 
                           "G-Force", "Battery Voltage", "Battery Current", "IMU X", "IMU Y", "IMU Z", "Cmd Received"])

# Initialize serial connection to read telemetry data
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Global flags and variables for telemetry handling
telemetry_paused = False  # Pauses telemetry reading during command send
prev_msg_timestamp = None

# =================== Main Window Creation ===================
# Create and configure the main telemetry display window
root = tk.Tk()
root.title("GCS Telemetry Visualiser")
root.attributes("-topmost", True)  # Ensure main window stays on top

# -------------------- GUI Labels Setup --------------------
labels_text = ["Timestamp", "Latency (ms)", "Freq (Hz)", "Angle of Attack", "Altitude", 
               "G-Force", "Battery Voltage", "Battery Current", "IMU X", "IMU Y", "IMU Z", "Cmd Received"]
labels = {}
for i, text in enumerate(labels_text):
    tk.Label(root, text=text+":", font=("Arial", 12)).grid(row=i, column=0, padx=5, pady=2, sticky="e")
    labels[text] = tk.Label(root, text="N/A", font=("Arial", 12))
    labels[text].grid(row=i, column=1, padx=5, pady=2, sticky="w")

# =================== Telemetry Update Function ===================
def update_gui():
    """Reads telemetry from serial port and updates GUI labels."""
    global prev_msg_timestamp
    if telemetry_paused:
        root.after(GUI_REFRESH_RATE, update_gui)
        return
    try:
        if ser.in_waiting:
            raw_line = ser.readline()
            line = raw_line.decode('utf-8', errors='replace').strip()
            parts = line.split(',')
            if len(parts) == 10:
                current_msg_timestamp = float(parts[0])
                latency = (time.time() - current_msg_timestamp) * 1000  
                if prev_msg_timestamp is not None:
                    elapsed = current_msg_timestamp - prev_msg_timestamp
                    freq = 1 / elapsed if elapsed > 0 else 0
                else:
                    freq = 0
                prev_msg_timestamp = current_msg_timestamp
                
                # Map telemetry values to corresponding labels
                data_map = {
                    "Timestamp": f"{parts[0]}",
                    "Latency (ms)": f"{latency:.0f}",
                    "Freq (Hz)": f"{freq:.1f}",
                    "Angle of Attack": parts[1],
                    "Altitude": parts[2],
                    "G-Force": parts[3],
                    "Battery Voltage": parts[4],
                    "Battery Current": parts[5],
                    "IMU X": parts[6],
                    "IMU Y": parts[7],
                    "IMU Z": parts[8],
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
    global telemetry_paused
    command = command_entry.get().strip()
    if command:
        telemetry_paused = True  # Pause telemetry reading during command send
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
