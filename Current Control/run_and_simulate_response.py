import numpy as np
import matplotlib.pyplot as plt
import serial
import csv
from ClassesAndFunctions import ZTranferFunction, HammersteinWiener
from collections import deque

# print available serial ports
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
print("Available serial ports:")
for port, desc, hwid in sorted(ports):
    print(f"{port}: {desc} [{hwid}]")

# print("Going to sleep...")
# while(1):
#     pass

# Define serial port and baud rate
serial_port = 'COM13'  # Change this to your serial port
baud_rate = 115200

# Open serial connection
ser = serial.Serial(serial_port, baud_rate)

# Initialize empty deque to store data
data_points_measured = deque(maxlen=1000)  # Adjust the maxlen as needed for your application
data_points_measured_real = deque(maxlen=1000)  # Adjust the maxlen as needed for your application
data_points_model = deque(maxlen=1000)  # Adjust the maxlen as needed for your application
data_points_time = deque(maxlen=1000)  # Adjust the maxlen as needed for your application
data_points_throttle_measured = deque(maxlen=1000)  # Adjust the maxlen as needed for your application

# Function to generate filename based on date and number run on that date
def generate_filename():
    import datetime
    import os
    today = datetime.date.today()
    file_num = 1
    while True:
        # data with underscores instead of -
        today = str(today).replace("-", "_")
        folder_name = f"measurements/{today}"
        file_name = f"{folder_name}/test_{file_num}.csv"
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
            return file_name
        if not os.path.exists(file_name):
            return file_name
        file_num += 1

file_name = generate_filename()
print(f"Saving data to {file_name}")
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

# Initialize plot
plt.ion()  # Turn on interactive mode
fig, axs = plt.subplots(2, 1, figsize=(10, 8))
line_throttle, = axs[0].plot([], [])  # Empty plot for now
line_measured, = axs[1].plot([], [], label="Measurement")  # Empty plot for now
line_measured_real, = axs[1].plot([], [], label="Real Measurement")  # Empty plot for now
line_model, = axs[1].plot([], [], label="Model Prediction")  # Empty plot for now

# Maximize the window on Windows OS
# Show the plot
plt.show(block=False)  # Open the plot window without blocking

# Get the current figure's manager
figManager = plt.get_current_fig_manager()

# Maximize the window using PyQt's method
if figManager.window is not None:
    figManager.window.showMaximized()
else:
    print("Failed to maximize the window: No window manager found.")


# Set plot labels and title
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Throttle')
axs[0].set_title('Throttle Signal')

axs[1].set_xlabel('Time')
axs[1].set_ylabel('Current')
axs[1].set_title('Real-time Serial Data Plot')

# Function to update plot
def update_plot():
    # line_measured.set_xdata(range(len(data_points_measured)))
    fig.canvas.flush_events()
    line_measured.set_xdata(data_points_time)
    line_measured.set_ydata(data_points_measured)

    # line_model.set_xdata(range(len(data_points_model)))
    line_model.set_xdata(data_points_time)
    line_model.set_ydata(data_points_model)

    line_throttle.set_xdata(data_points_time)
    line_throttle.set_ydata(data_points_throttle_measured)

    line_measured_real.set_xdata(data_points_time)
    line_measured_real.set_ydata(data_points_measured_real)

    axs[0].relim()
    axs[0].autoscale_view()
    axs[0].set_ylim(0, 1)
    axs[1].relim()
    axs[1].autoscale_view()
    axs[1].set_ylim(-1, 32)
    fig.canvas.draw()

iter_num = 0
try:
    while True:
        # Read serial input
        serial_data = ser.readline().decode().strip()
        if iter_num > 10:
            
            # Print received data
            print(serial_data)

            # get throttle value from that string
            thr_ = serial_data.split(",")
            # thr = int(thr_[1])
            thr = thr_[1]

            c_measured = float(thr_[5])
            # c_measured_real = 0
            c_measured_real = float(thr_[2])

            u = float((int(thr) - 1000)) / 1000
            # c_predicted = model(u)
            c_predicted = 0
            model_response = np.append(model_response, c_predicted)

            # Update plot
            data_points_measured.append(c_measured)
            data_points_measured_real.append(c_measured_real)
            data_points_model.append(c_predicted)
            data_points_time.append(int(thr_[0])/1e3)
            data_points_throttle_measured.append(float(u))
            
            if iter_num % 70 == 0:
                update_plot()

            # Write data to CSV file
            csv_writer.writerow([serial_data, c_predicted])
        iter_num += 1

except KeyboardInterrupt:
    print("Keyboard Interrupt detected. Closing serial connection.")
    ser.close()
    csv_file.close()
    
    import os
    file_name_no_ext = file_name.split(".")[0]

    # Ask for save
    save_q = input("Do you want to save the experiment? [/n]: ")
    if save_q.lower() == "n":
        print("Experiment not saved. Exiting...")
        # remove the file
        os.remove(f"{file_name_no_ext}.csv")
    else:
        # Close serial connection and CSV file when KeyboardInterrupt (Ctrl+C) is detected
        print(f"Saving data to {file_name}...")
        
        # Run convert__data_for_matlab.py with file_name as argument
        os.system(f"python data_collection/convert_data_for_matlab.py \"{file_name_no_ext}\"")

        # ask for user input notes to the experiment, if given write to txt file, else if just pressed enter dont
        notes = input("Enter notes for the experiment: ")
        if notes:
            with open(f"{file_name_no_ext}.txt", "w") as f:
                f.write(notes)
            print(f"Notes saved to {file_name_no_ext}.txt")


