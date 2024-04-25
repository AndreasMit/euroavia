import numpy as np
import matplotlib.pyplot as plt

file_name = "measurements/test_with_simulation_25A_Working_mat.csv"
# file_name = "measurements/test5.csv"

# Load data from CSV file
data = np.genfromtxt(file_name, delimiter=',', dtype=str, skip_header=6)

# Remove double quotes from each element in the array
data = np.char.strip(data, '"')
# Convert array to float
data = data.astype(float)

plt.figure()
plt.title("Input")
plt.plot(data[:, 0], data[:, 1], 'b-')
plt.grid()

plt.figure()
plt.title("Output")
plt.plot(data[:, 0], data[:, 5], 'r-')
plt.plot(data[:, 0], data[:, 2], 'b-')
plt.axhline(y=25, color='k', linestyle='--', alpha=0.5, label='Target')
plt.grid()

plt.figure()
plt.title("Sampling Frequency")
plt.plot(data[:, 0], data[:, 3], 'b-')
plt.grid()

# Mean Sampling Freq and Std
mean_sampling_freq = np.mean(data[:, 3])
std_sampling_freq = np.std(data[:, 3])
print(f"Mean Sampling Frequency: {mean_sampling_freq:.2f} Hz")
print(f"Std Sampling Frequency: {std_sampling_freq:.2f} Hz")

# print(f"Mean Delta Time: {1/mean_sampling_freq:.2f} s")
# print(f"Std Delta Time: {1/std_sampling_freq:.2f} s")
# print(f"Min Delta Time: {1/np.max(data[:, 3]):.2f} s")
# print(f"Max Delta Time: {1/np.min(data[:, 3]):.2f} s")

plt.show()
