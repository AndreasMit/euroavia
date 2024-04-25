import numpy as np
import matplotlib.pyplot as plt

# "Time[ms], Throttle[1000 -> 2000], Current_MA[A], Frequency [Hz], Control Enabled Flag, Current Raw[A]"
file_name = "measurements/2024_04_25/test_31_mat.csv"

# Load data from CSV file
data = np.genfromtxt(file_name, delimiter=',')


time_ms = data[:, 0]
time_s = time_ms / 1000
thr = data[:, 1]
current_ma = data[:, 2]
freq_measured = data[:, 3]
c_enabled_flag = data[:, 4]
current_raw = data[:, 5]
int_sum = (data[:, 6] * 0.055) * 1000 + 1000
test_var = data[:, 7]
kp_part = np.clip(thr - int_sum - 800, 0, 1000) + 1000
# keep only the kp_part where test_var == false, else = 1000
kp_part = np.where(c_enabled_flag == 0, kp_part, 1000)

# Plot data
fig, axs = plt.subplots(2, 1, figsize=(10, 7))
axs[0].plot(time_s, thr, 'b-')
axs[0].plot(time_s, int_sum, 'r-')
axs[0].plot(time_s, test_var*1000 + 1000, 'g-')
axs[0].plot(time_s, kp_part, 'y-')
axs[0].set_title("Input")
axs[0].grid()
axs[0].set_ylim([900, 2100])

axs[1].plot(time_s, current_raw, 'r-', label="Raw Current")
axs[1].plot(time_s, current_ma, 'b-', label="Filtered ARD")
axs[1].plot(time_s, test_var*25, 'g-', label="Test Var")
axs[1].axhline(y=25, color='k', linestyle='--', alpha=0.5, label='Target')
axs[1].set_title("Output")
axs[1].set_ylabel("Current [A]")
axs[1].set_xlabel("Time [s]")
axs[1].legend()
axs[1].grid()

plt.figure()
plt.title("Sampling Frequency")
plt.plot(time_ms, freq_measured, 'b-')
plt.grid()

# Mean Sampling Freq and Std
mean_sampling_freq = np.mean(freq_measured)
std_sampling_freq = np.std(freq_measured)
print(f"Mean Sampling Frequency: {mean_sampling_freq:.2f} Hz")
print(f"Std Sampling Frequency: {std_sampling_freq:.2f} Hz")


plt.show()
