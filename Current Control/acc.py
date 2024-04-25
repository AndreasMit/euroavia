from myFunctions import *

data = np.genfromtxt("test_with_simulation_25A_Working_new_mat.csv", delimiter=",")

time = data[:, 0]
y = data[:, 5]
y2 = data[:, 2]

fft, ff = fftTransform(y, 100)
fft2, ff2 = fftTransform(y2, 100)

# plot time series
plotData(0, time, y, label=f"Time Series Raw", timeFlag=True, multiSelectFlag=False, c='k')
plotData(0, time, y2, label=f"Time Series Raw 2", timeFlag=True, multiSelectFlag=False, c='r')

plotData(1, ff, fft, label=f"FFT", c='k', multiSelectFlag=False)
plotData(1, ff2, fft2, label=f"FFT2", c='r', multiSelectFlag=False)
plt.show()