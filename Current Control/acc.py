from myFunctions import *

data = np.genfromtxt("case_rs_6689.csv", delimiter=",")

time_s = data[:, 0] / 1e6
throttle_pos = data[:, 1]
c_measured = data[:, 2]

# keep only the part of the arrays from time = 2 seconds and on
indices = time_s >= 21
time_s = time_s[indices]
throttle_pos = throttle_pos[indices]
c_measured = c_measured[indices]


freq = np.zeros(time_s.shape)
for i in range(1, len(time_s)):
    freq[i] = 1 / (time_s[i] - time_s[i-1])

# print mean and std
print(f"Mean Sampling Frequency: {np.mean(freq):.2f} Hz")
print(f"Std Sampling Frequency: {np.std(freq):.2f} Hz")


time_s = time_s[1:]
throttle_pos = throttle_pos[1:]
c_measured = c_measured[1:]


fft, ff = fftTransform(c_measured, 6689)

# plot time series
plotData(0, time_s, c_measured, label=f"Time Series Raw", timeFlag=True, multiSelectFlag=False, c='k')

plotData(1, ff, fft, label=f"FFT", c='k', multiSelectFlag=False)

plotData(2, time_s, throttle_pos, label=f"Throttle Position", timeFlag=True, multiSelectFlag=False, c='r')

# plot freq
plotData(3, time_s, freq[1:], label=f"Sampling Frequency", timeFlag=True, multiSelectFlag=False, c='b')

plt.show()