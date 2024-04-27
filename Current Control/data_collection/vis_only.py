import numpy as np
import matplotlib.pyplot as plt

# "Time[ms], Throttle[1000 -> 2000], Current_MA[A], Frequency [Hz], Control Enabled Flag, Current Raw[A]"
file_name = "measurements/2024_04_27/test_22_mat.csv"

# Load data from CSV file
data = np.genfromtxt(file_name, delimiter=',')


time_ms = data[:, 0]
time_s = time_ms / 1000
thr = data[:, 1]
current_ma = data[:, 2]
freq_measured = data[:, 3]
c_enabled_flag = data[:, 4]
current_raw = data[:, 5]
# ki_part = data[:, 6] * 1000 + 1000
# kp_part = data[:, 7] * 1000 + 1000
# kd_part = data[:, 8] * 1000 + 1000
ki_part = 0
kp_part = 0
kd_part = 0
test_var = 0

# Plot data
fig, axs = plt.subplots(2, 1, figsize=(10, 7))
axs[0].plot(time_s, thr, 'k-', label="Throttle")
# axs[0].plot(time_s, ki_part, 'r-', label="Ki Part")
# axs[0].plot(time_s, kp_part, 'b-', label="Kp Part")
# axs[0].plot(time_s, kd_part, 'g-', label="Kd Part")
# axs[0].plot(time_s, test_var*1000 + 1000, 'g-')
axs[0].fill_between(time_s, 1000 + c_enabled_flag*500,1000, color='lightgreen', alpha=0.35, label="Switched Control?")

axs[0].set_ylabel("Throttle PWM") 
axs[0].grid()
# axs[0].set_xlim([20, 30])
axs[0].set_ylim([900, 2100])
axs[0].legend()

# axs[1].plot(time_s, current_raw, 'r-', label="Raw Current")
axs[1].plot(time_s, current_ma, 'k-', label="Current Measurement")
# axs[1].plot(time_s, test_var*25, 'g-', label="Test Var")
axs[1].axhline(y=25, color='k', linestyle='--', alpha=0.5, label='Target')
axs[1].axhline(y=20, color='k', linestyle='--', alpha=0.5)
# Control enabled flag
axs[1].fill_between(time_s, c_enabled_flag * 15, color='lightgreen', alpha=0.35)
axs[1].set_ylabel("Current [A]")
axs[1].set_xlabel("Time [s]")
# axs[1].set_xlim([20, 30])
axs[1].set_ylim([-5, 40])

# legend outside of the area
# plt.legend(loc='upper right', bbox_to_anchor=(1, 1))
axs[1].grid()
axs[1].legend()

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
