import numpy as np

# input_file = "measurements/test_with_simulation_mat.csv"
# output_file = "measurements/test_with_simulation_rs_10ms.csv"
input_file = "case.csv"
output_file = "case_rs_6689.csv"

# Load the data from CSV file
data = np.loadtxt(input_file, delimiter=',')

# Extract time, out, and in from the loaded data
time = data[:, 0]
inn = data[:, 1]
out = data[:, 5]

# Calculate the time intervals between adjacent points
time_intervals = np.diff(time)
print(f"Mean time interval: {np.mean(time_intervals):.2f} ms \t [{np.min(time_intervals):.2f}, {np.max(time_intervals):.2f}]")
print(f"Std time interval: {np.std(time_intervals):.2f} ms")


# Define the constant time interval (deltat)
deltat = 1/6689*1000 # milliseconds
# target_freq = 55  # Hz
# deltat = 1 / target_freq * 1000  # milliseconds

# Calculate the number of points with the constant time interval
num_points = int((time[-1] - time[0]) / deltat)

# Create a new time array with constant time intervals
new_time = np.linspace(time[0], time[-1], num_points)

# Interpolate 'out' and 'in' values linearly
out_interp = np.interp(new_time, time, out)
in_interp = np.interp(new_time, time, inn)

print(f"Inp shape: {in_interp.shape}")
print(f"Out shape: {out_interp.shape}")

# Stack the interpolated time, out, and in values horizontally
resampled_data = np.column_stack((new_time, out_interp, in_interp))

# Save the resampled data to a new CSV file
np.savetxt(output_file, resampled_data, delimiter=',', header='time,in,out', comments='')
print(f"Resampled data saved to {output_file}")


# Plot the original and resampled data for comparison
import matplotlib.pyplot as plt

plt.figure()
plt.plot(time, out, 'b.-', label='Original')
plt.plot(new_time, out_interp, 'r.-', label='Resampled')
plt.grid()
plt.legend()
plt.show()