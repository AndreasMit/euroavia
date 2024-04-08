import numpy as np
import matplotlib.pyplot as plt
import serial
import csv
from model_sym_test import ZTranferFunction, HammersteinWiener


# Define serial port and baud rate
serial_port = 'COM8'  # Change this to your serial port
baud_rate = 115200

# Open serial connection
ser = serial.Serial(serial_port, baud_rate)

# Open CSV file for writing
file_name = "measurements/test_with_simulation.csv"
csv_file = open(file_name, 'w', newline='')
csv_writer = csv.writer(csv_file)

# Define Model ---------------------------------------------------
# Input Nonlinearity        
x_inp = [-0.4731, 0.2448, 0.419, 0.4531, 0.6003, 0.8796, 1]
y_inp = [-0.039, 0.0438, 0.1095, 0.1460, 0.2493, 0.5134, 0.627248]
# Output Nonlinearity
x_out = [-4.1721, -2.6285, -0.7644, 0.0366, 0.1922, 1.071, 2.2186, 3.0185, 3.0625, 3.6097, 6.8054]
y_out = [-3.1008, -2.8004, 0.2749, -0.0683, 0.0711, 5.0916, 14.0317, 22.6874, 22.5085, 25.7928, 49.701]

# Define Model
model_ = ZTranferFunction([1, -0.9605], [1, -2.557, +2.214, -0.6516])
model  = HammersteinWiener(model_, np.array([x_inp, y_inp]), np.array([x_out, y_out]))
# -----------------------------------------------------------------

model_response = np.array([])

iter_num = 0
try:
    while True:
        # Read serial input
        serial_data = ser.readline().decode().strip()
        
        # Print received data
        print(serial_data)
        if iter_num > 6:
            
            # get throttle value from that string
            thr_1 = serial_data.split(",")
            thr = int(thr_1[1])

            u = float(thr - 1000) / 1000
            y = model(u)
            # print(f"Input: {u:.2f}, Output: {y:.2f}")
            model_response = np.append(model_response, y)


        iter_num += 1
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

plt.figure(1)
plt.title("Input")
plt.step(data[:, 0], data[:, 1], 'b-')
plt.grid()

plt.figure(2)
plt.title("Output")
plt.step(data[:, 0], data[:, 2], 'b-', label='Measured Output')
plt.step(data[:, 0][:-1], model_response, 'r-', label='Model Output')
plt.grid()
plt.show()
