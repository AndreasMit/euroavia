import numpy as np
import matplotlib.pyplot as plt
from myFunctions import *

data = np.genfromtxt("data.csv", skip_header=3, delimiter=",")
# read header (3rd row)
header = np.genfromtxt("data.csv", skip_footer=len(data), delimiter=",", dtype=str)
header = header[0]
print(header)

# from data[:, 3] keep only the numbers that are not nan
data = data[~np.isnan(data[:, 3])]

# print f sample (first column is time in ms)
print("f sample: ", 1/(data[1,0]-data[0,0])*1000)


# plot time with pwm and current (A)
plt.figure(1)
plt.plot(data[:,0]/1000, data[:,3], label=header[3])
sg = savgovFilter(data[:, 3], 30, 0)
plt.plot(data[:,0]/1000, sg, 'r-', label="Filtered ")
sg = savgovFilter(sg, 30, 1)
plt.plot(data[:,0]/1000, sg, 'k-', label="Filtered 2")
plt.plot(data[:,0]/1000, data[:, 2]/2000*3, label=header[2])
plt.xlabel("Time (s)")
plt.legend()

# print mean and std
print("Mean: ", np.mean(data[:,3]))
print("Std: ", np.std(data[:,3]))

# horizontal line at mean
plt.axhline(y=np.mean(data[:,3]), color='r', linestyle='-')
# horizontal line at mean +/- std
plt.axhline(y=np.mean(data[:,3]) + np.std(data[:,3]), color='g', linestyle='-')
plt.axhline(y=np.mean(data[:,3]) - np.std(data[:,3]), color='g', linestyle='-')

# put mean and std in title (3 decimal places)
plt.title("Mean: " + str(np.round(np.mean(data[:,3]), 3)) + " Std: " + str(np.round(np.std(data[:,3]), 3)))

# plt.show()


# fft 
fft, ff = fftTransform(sg, 50)
plotData(2, ff, fft, fftFlag=True)
# plt.show()

# plot data[:, 5 -> 8]
plt.figure(3)
plt.plot(data[:,0]/1000, data[:,5], label=header[5])
plt.plot(data[:,0]/1000, data[:,6], label=header[6])
plt.plot(data[:,0]/1000, data[:,7], label=header[7])
plt.plot(data[:,0]/1000, data[:,8], label=header[8])
plt.xlabel("Time (s)")
plt.legend()
# plt.show()

# plot their sg filtered version
plt.figure(4)
plt.plot(data[:,0]/1000, savgovFilter(data[:,5], 30, 0), label=header[5])
plt.plot(data[:,0]/1000, savgovFilter(data[:,6], 30, 0), label=header[6])
plt.plot(data[:,0]/1000, savgovFilter(data[:,7], 30, 0), label=header[7])
plt.plot(data[:,0]/1000, savgovFilter(data[:,8], 30, 0), label=header[8])
plt.xlabel("Time (s)")
plt.legend()
plt.show()
