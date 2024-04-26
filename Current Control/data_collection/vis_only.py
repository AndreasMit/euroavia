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

# Plot data
fig, axs = plt.subplots(2, 1, figsize=(10, 7))
axs[0].plot(time_s, thr, 'k-', label="Throttle")
# axs[0].plot(time_s, int_sum, 'r-')
# axs[0].plot(time_s, test_var*1000 + 1000, 'g-')
axs[0].fill_between(time_s, 1000 + c_enabled_flag*1000,1000, color='lightgreen', alpha=0.35, label="Switched Control?")

axs[0].set_ylabel("Throttle PWM") 
axs[0].grid()
axs[0].set_xlim([148, 207])
axs[0].set_ylim([900, 2100])
axs[0].legend()

# axs[1].plot(time_s, current_raw, 'r-', label="Raw Current")
axs[1].plot(time_s, current_ma, 'k-', label="Current Measurement")
# axs[1].plot(time_s, test_var*25, 'g-', label="Test Var")
axs[1].axhline(y=25, color='k', linestyle='--', alpha=0.5, label='Target')
# Control enabled flag
axs[1].fill_between(time_s, c_enabled_flag * 50, color='lightgreen', alpha=0.35)
axs[1].set_ylabel("Current [A]")
axs[1].set_xlabel("Time [s]")
axs[1].set_xlim([148, 207])
axs[1].set_ylim([-5, 40])

# legend outside of the area
# plt.legend(loc='upper right', bbox_to_anchor=(1, 1))
axs[1].grid()
axs[1].legend()

# plt.figure()
# plt.title("Sampling Frequency")
# plt.plot(time_ms, freq_measured, 'b-')
# plt.grid()

# Mean Sampling Freq and Std
mean_sampling_freq = np.mean(freq_measured)
std_sampling_freq = np.std(freq_measured)
print(f"Mean Sampling Frequency: {mean_sampling_freq:.2f} Hz")
print(f"Std Sampling Frequency: {std_sampling_freq:.2f} Hz")


plt.show()
