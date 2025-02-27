import tkinter as tk
import serial
import time
import csv                  # added for CSV writing
from datetime import datetime  # added for timestamped filename

# Configure Parameters ------------------------

SERIAL_PORT = 'COM12'   # Adjust as needed
BAUD_RATE = 9600
GUI_REFRESH_RATE = 100  # milliseconds

SAVE_TO_FILE = False    # Set to True to save data to file

# -----------------------------------------------------

# Labels for GUI fields
labels_text = ["Timestamp", "Latency (ms)", "Freq (Hz)", "Angle of Attack", "Altitude", 
                "G-Force", "Battery Voltage", "Battery Current", "IMU X", "IMU Y", "IMU Z"]

# Initialize CSV file if saving is enabled
if SAVE_TO_FILE:
    start_time = datetime.now()
    filename = start_time.strftime("%Y%m%d_%H%M%S") + ".csv"
    csv_file = open(filename, "w", newline="")
    csv_writer = csv.writer(csv_file)
    # Write header row
    csv_writer.writerow(labels_text)

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Create the main window
root = tk.Tk()
root.title("GCS Telemetry Visualiser")

labels = {}
for i, text in enumerate(labels_text):
    tk.Label(root, text=text+":", font=("Arial", 12)).grid(row=i, column=0, padx=5, pady=2, sticky="e")
    labels[text] = tk.Label(root, text="N/A", font=("Arial", 12))
    labels[text].grid(row=i, column=1, padx=5, pady=2, sticky="w")

# Global variable to store the previous message timestamp for frequency calculation
prev_msg_timestamp = None

def update_gui():
    global prev_msg_timestamp
    try:
        # Read a line from serial port
        if ser.in_waiting:
            raw_line = ser.readline()
            # print("Debug: Received message size:", len(raw_line), "bytes")
            line = raw_line.decode('utf-8', errors='replace').strip()
            # Expecting CSV: timestamp,angle,alt,force,voltage,current,imu_x,imu_y,imu_z
            parts = line.split(',')
            if len(parts) == 9:
                current_msg_timestamp = float(parts[0])
                # Calculate latency in milliseconds based on system time
                latency = (time.time() - current_msg_timestamp) * 1000  
                # Calculate frequency if a previous timestamp exists
                if prev_msg_timestamp is not None:
                    elapsed = current_msg_timestamp - prev_msg_timestamp
                    freq = 1 / elapsed if elapsed > 0 else 0
                else:
                    freq = 0
                prev_msg_timestamp = current_msg_timestamp
                
                # Build the mapping for GUI labels
                data_map = {
                    "Timestamp": f"{current_msg_timestamp:.0f}",
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
                }
                for key, val in data_map.items():
                    labels[key].config(text=val)
                # Append the data row to CSV if enabled
                if SAVE_TO_FILE:
                    csv_writer.writerow([data_map[key] for key in labels_text])
                    csv_file.flush()
    except Exception as e:
        print("Error reading serial:", e)
    root.after(GUI_REFRESH_RATE, update_gui)  # schedule next read

# Start the update loop
update_gui()

# Run the GUI main loop
root.mainloop()
