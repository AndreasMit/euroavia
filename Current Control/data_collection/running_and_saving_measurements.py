import numpy as np
import matplotlib.pyplot as plt
import serial
import csv

# Define serial port and baud rate
serial_port = 'COM8'  # Change this to your serial port
baud_rate = 115200

# Open serial connection
ser = serial.Serial(serial_port, baud_rate)

# Open CSV file for writing
file_name = "measurements/test_with_rc_1.csv"
csv_file = open(file_name, 'w', newline='')
csv_writer = csv.writer(csv_file)

try:
    while True:
        # Read serial input
        serial_data = ser.readline().decode().strip()
        
        # Print received data
        print(serial_data)

        # Write data to CSV file
        csv_writer.writerow([serial_data])

except KeyboardInterrupt:
    # Close serial connection and CSV file when KeyboardInterrupt (Ctrl+C) is detected
    ser.close()
    csv_file.close()

# Plotting Results -------------------------------

# Load data from CSV file
data = np.genfromtxt(file_name, delimiter=',', dtype=str, skip_header=6)

# Remove double quotes from each element in the array
data = np.char.strip(data, '"')
# Convert array to float
data = data.astype(float)

print(data)
print(data.shape)

plt.figure()
plt.title("Input")
plt.plot(data[:, 0], data[:, 1], 'b-')
plt.grid()

plt.figure()
plt.title("Output")
plt.plot(data[:, 0], data[:, 2], 'b-')
plt.grid()
plt.show()
