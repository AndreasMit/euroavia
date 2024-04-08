
'''
Usage Syntax of the functions:

    - fftTransform(y, fs)   ->  signal_fft_half, ff_half
    - hilbertTransform(y)   ->  signal_envelope, signal_phase

    - butterworthFilter(y, fs, fL, fH, order)   ->  filtered_signal
    - savgovFilter(y, order=2, framelen=299)    ->  signal_filtered
    - wden(y, level=5, wname='sym8')            ->  signal_denoised
    
    - calculateOA(y, fs)    ->  OA

'''

# FFT SIGNAL -----------------------------------------------------------------

import numpy as np

def fftTransform(signal, fs):
    # Convert to m/s^2 rms
    # signal = 9.81 * 0.707 * signal

    # FFT
    signal_fft = np.fft.fft(signal)
    signal_fft = np.abs(signal_fft) / len(signal)
    signal_fft_half = signal_fft[:len(signal)//2]
    signal_fft_half = signal_fft_half / np.max(signal_fft_half)
    
    # Frequency array
    ff = np.fft.fftfreq(len(signal), 1 / fs)
    ff_half = ff[:len(ff)//2]  # Take only the positive frequencies

    # Set DC component (first element) to zero
    signal_fft_half[0] = 0
    
    return signal_fft_half, ff_half







# STFT -----------------------------------------------------------------------

# import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import stft
import mplcursors

"""
    Compute the Short-Time Fourier Transform (STFT) of a given signal.

    Parameters:
        - data: array_like
            The input signal.
        - fs: int
            The sampling frequency of the input signal.
        - nperseg: int, optional
            The length of each segment for the STFT calculation. Default is 256.
        - noverlap: int, optional
            The number of samples to overlap between segments. Default is 128.
        - nfft: int, optional
            The number of points to compute the FFT. Default is 256.
        - plot3dFlag: bool, optional
            Flag indicating whether to plot the STFT in 3D. Default is False.
        - plot2dFlag: bool, optional
            Flag indicating whether to plot the STFT in 2D. Default is False.
        - title: str, optional
            Title for the plot. Default is None.
        - convToDbFlag: bool, optional
            Flag indicating whether to convert the STFT magnitude to dB scale. Default is False.

    Returns:
        - f: ndarray
            Array of sample frequencies.
        - t: ndarray
            Array of segment times.
        - Zxx: ndarray
            STFT of the input signal.

"""

def stftTransform(data, fs, nperseg=256, noverlap=128, nfft=256, plot3dFlag=False, plot2dFlag=False, title=None, convToDbFlag=False, figNumber=100):
    # STFT
    f, t, Zxx = stft(data, fs, nperseg=nperseg, noverlap=noverlap, nfft=nfft)
    
    # Plot STFT
    if plot2dFlag:
        fig, ax = plt.subplots()
        if not convToDbFlag:
            im = ax.pcolormesh(t, f, np.abs(Zxx), shading='auto')
        else:
            im = ax.pcolormesh(t, f, 10 * np.log10(np.abs(Zxx)), shading='auto')
        plt.ylabel('Frequency [Hz]')
        plt.xlabel('Time [sec]')
        plt.colorbar(im)
        if title:
            plt.title(f"STFT Magnitude ({title})")
        else:
            plt.title('STFT Magnitude')


    if plot3dFlag:
        # Create a 3D plot
        fig = plt.figure(figNumber)
        ax = fig.add_subplot(111, projection='3d')

        # Plot the spectrogram in 3D
        X, Y = np.meshgrid(t, f)
        if not convToDbFlag:
            ax.plot_surface(Y, X, np.abs(Zxx), cmap='viridis')
        else:
            ax.plot_surface(Y, X, 10 * np.log10(np.abs(Zxx)), cmap='viridis')

        # Set labels and title
        ax.set_ylabel('Time [sec]')
        ax.set_xlabel('Frequency [Hz]')
        if convToDbFlag:
            ax.set_zlabel('Power/Frequency [dB/Hz]')
        else:
            ax.set_zlabel('Power/Frequency')
        if title:
            ax.set_title(f"3D Spectrogram ({title})")
        else:
            ax.set_title('3D Spectrogram')

    # plt.show()

    return f, t, Zxx








# HILBERT TRANSFORM ----------------------------------------------------------

# import numpy as np
from scipy.signal import hilbert

def hilbertTransform(signal):
    # Hilbert transform
    signal_hilbert = hilbert(signal)
    signal_envelope = np.abs(np.imag(signal_hilbert))
    signal_phase = np.unwrap(np.angle(signal_hilbert))
    
    return signal_envelope, signal_phase







# BUTTERWORTH FILTER ---------------------------------------------------------

# import numpy as np
from scipy.signal import butter, filtfilt

def butterworthFilter(signal, fs, fLow, fHigh, order, mode="pass"):
    # Normalize the frequencies by the Nyquist frequency (half the sampling rate)
    Wn = [2 * fLow / fs, 2 * fHigh / fs]

    # Create a bandpass Butterworth filter
    if mode == "pass":
        b, a = butter(order, Wn, btype='band')
    elif mode == "stop":
        b, a = butter(order, Wn, btype='bandstop')

    # Apply the filter to the signal using filtfilt (zero-phase filtering)
    filtered_signal = filtfilt(b, a, signal)

    return filtered_signal


# MOVING AVERAGE FILTER -------------------------------------------------------
# import numpy as np

def movingAverageFilter(signal, window):
    # Create a moving average filter
    moving_avg_filter = np.ones(window) / window
    # Apply the filter to the signal
    signal_filtered = np.convolve(signal, moving_avg_filter, mode='valid')

    return signal_filtered



# SAVGOV FILTER (Savitzky-Golay) ---------------------------------------------

# import numpy as np
from scipy.signal import savgol_filter

def savgovFilter(signal, window_size = 299, order = 2):

    # Apply the Savitzky-Golay filter
    signal_filtered = savgol_filter(signal, window_size, order)
    
    return signal_filtered





# WAVELET DENOISING ---------------------------------------------------------

import pywt
# import numpy as np

# Decompose the signal using wavelet decomposition
''' Parameters for Wavelet denoising
    lev = 5  # Level of decomposition
    wname = 'sym8'  # Type of wavelet
'''
def wden(signal, level=5, wname='sym8'):
    # Decompose the signal using wavelet decomposition
    coeffs = pywt.wavedec(signal, wname, level=level)
    # Thresholding the detail coefficients
    sigma = np.median(np.abs(coeffs[-1])) / 0.6745
    uthresh = sigma * np.sqrt(2 * np.log(len(signal)))
    for i in range(1, len(coeffs)):
        coeffs[i] = pywt.threshold(coeffs[i], value=uthresh, mode='soft')
    # Reconstruct the signal from the thresholded coefficients
    signal_denoised = pywt.waverec(coeffs, wname)
    return signal_denoised







# CALCULATE OVERALL AMPLITUDE -------------------------------------------------
# import numpy as np

def calculateOA(signal, fs):
    # Convert to m/s^2 rms
    signal = 9.81 * 0.707 * signal

    # FFT
    signal_fft = np.fft.fft(signal)
    signal_fft = np.abs(signal_fft) / len(signal)
    
    # Frequency array
    ff = np.fft.fftfreq(len(signal), 1 / fs)
    ff_half = ff[:len(ff)//2]  # Take only the positive frequencies

    # Set DC component (first element) to zero
    signal_fft[0] = 0
    signal_fft_half = signal_fft[:len(ff)//2]

    # Overall amplitude calculation
    OA = (1000/(2*np.pi)) * np.sqrt(np.sum(np.square(signal_fft_half[10:1001]) / np.square(ff_half[10:1001])))
    
    return OA


# INTEGRATE SIGNAL -----------------------------------------------------------
# import numpy as np

# Return the integral of the signal
def integrateSignal(signal, time_signal, removeBestFitLineFlag=True):
    integ_signal = 0
    # Calculate the integral of the signal
    integ_signal = [np.trapz(signal[:i], time_signal[:i]) for i in range(len(signal))]
    # substraction of the mean value
    integ_signal = integ_signal - np.mean(integ_signal)

    if removeBestFitLineFlag:
        integ_signal = removeBestFitLine(integ_signal, time_signal)

    return np.array(integ_signal)

def removeBestFitLine(signal, time_signal):
    # Fit a line to the signal
    best_fit_line = np.poly1d(np.polyfit(time_signal, signal, 1))
    # Remove the best fit line from the integral
    signal = signal - best_fit_line(time_signal)

    return signal







# CUSTOM PLOT FUNCTION --------------------------------------------------------
# import numpy as np
# import matplotlib.pyplot as plt
# import mplcursors


"""
Plots a signal with optional features such as labeling, color selection, and interactive annotation.

Parameters:
- figNum (int):                     The figure number for the plot.
- x (array-like):                   The x-coordinates of the signal.
- y (array-like):                   The y-coordinates of the signal.
- label (str):                      The label for the signal.
- c (str, optional):                The color of the signal. Default is 'k' (black).
- timeFlag (bool, optional):        Flag indicating if the x-axis represents time. Default is False.
- fftFlag (bool, optional):         Flag indicating if the x-axis represents frequency for FFT analysis. Default is False.
- hilbFlag (bool, optional):        Flag indicating if the x-axis represents frequency for Hilbert analysis. Default is False.
- multiSelectFlag (bool, optional): Flag indicating if multiple points can be selected for annotation. Default is False.

Returns:
None
"""

# Print delta t and freq difference
def printDistanceClicked(x_coords, timeFlag=False):
    if len(x_coords) > 1:
        x1 = x_coords[-2]
        x2 = x_coords[-1]
        if timeFlag:
            print(f"Freq Dist ({x1:.5f} -> {x2:.5f}):\t{1/(x2-x1):.1f} Hz")
        else:
            print(f"Freq Dist ({x1:.1f} -> {x2:.1f}):\t{(x2-x1):.1f} Hz")


def plotData(figNum, x, y, label = None, xlim=None, lw=1.5, c="k", timeFlag=False, fftFlag=False, hilbFlag=False, multiSelectFlag=False, title=None, alpha=1):
    
    plt.figure(figNum)
    line, = plt.plot(x, y, color=c, label=label, linewidth=lw, alpha=alpha)

    if timeFlag:
        plt.xlabel('Time [s]')
        if not title:
            plt.title('Time Domain')
    elif fftFlag:
        plt.xlabel('Frequency [Hz]')
        if not title:
            plt.title('Frequency Domain (FFT)')
    elif hilbFlag:
        plt.xlabel('Frequency [Hz]')
        if not title:
            plt.title('Frequency Domain (Hilbert)')
    if title:
        plt.title(title)
    
    if xlim:
        plt.xlim(xlim)

    plt.ylabel('Amplitude')
    plt.legend()
    plt.grid()
    plt.tight_layout()


    # Connect the click event handler
    x_clicked = []
    if multiSelectFlag:
        annotations = []
        # Function to handle click events
        def on_click_multi(sel):
            # Get the x-coordinate of the clicked point
            x_coord = sel.target[0]
            x_clicked.append(x_coord)
            # Create a new annotation
            annotation = plt.annotate(f'{x_coord:.0f} Hz', xy=(x_coord, sel.target[1]),
                                    xytext=(10, 10), textcoords="offset points",
                                    bbox=dict(boxstyle="round,pad=0.5", fc="white", alpha=0.6))
            annotations.append(annotation)
            printDistanceClicked(x_clicked, timeFlag)

            plt.draw()
        mplcursors.cursor(line, hover=False).connect("add", on_click_multi)
    else:
        # Function to handle click events
        def on_click(sel):
            # Get the x-coordinate of the clicked point
            x_coord = sel.target[0]
            x_clicked.append(x_coord)
            # Display a text box with the x-coordinate
            sel.annotation.set_text(f'{x_coord:.0f} Hz')
            sel.annotation.get_bbox_patch().set(fc="white", alpha=0.6)
            printDistanceClicked(x_clicked, timeFlag)
        mplcursors.cursor(line, hover=False).connect("add", on_click)


# PLOT ORBIT -----------------------------------------------------------------
# import numpy as np
# import matplotlib.pyplot as plt
import matplotlib.animation as animation

"""
Plots the orbit of two-dimensional data points.

Parameters:
    - data1 (array-like): The first set of data points.
    - data2 (array-like): The second set of data points.
    - SKIP_FIRST_N_POINTS (int): The number of initial points to skip. Default is 0.
    - SKIP_FRAMES_STEP (int): The step size for skipping frames. Default is 1.
    - NUM_OF_FRAMES_TO_USE (int): The number of frames to use for plotting. Default is 16384.
    - PLOT_SNAKE_LIKE_FLAG (bool): Flag indicating whether to plot the orbit as a snake-like line. Default is True.
    - SNAKE_SIZE (int): The number of points to connect when plotting the snake-like line. Default is 5.
    - x_title (str): The title for the x-axis. Default is "X".
    - y_title (str): The title for the y-axis. Default is "Y".
    - ANIM_INTERVAL (int): The interval between frames in milliseconds. Default is 100.
    - SCATTER_FLAG (bool): Flag indicating whether to plot the points as scatter plot. Default is False.
"""

def plotOrbit(data1, data2, SKIP_FIRST_N_POINTS = 0, SKIP_FRAMES_STEP=1, NUM_OF_FRAMES_TO_USE=16384, PLOT_SNAKE_LIKE_FLAG = True, SNAKE_SIZE=5, x_title="X", y_title="Y", ANIM_INTERVAL=100, SCATTER_FLAG=False):

    # Crop data
    data1 = data1[SKIP_FIRST_N_POINTS:NUM_OF_FRAMES_TO_USE]
    data2 = data2[SKIP_FIRST_N_POINTS:NUM_OF_FRAMES_TO_USE]

    # ranges as writen before
    x_range = max([np.max(data1), np.abs(np.min(data1))])
    y_range = max([np.max(data2), np.abs(np.min(data2))])

    fig, ax = plt.subplots(1, 1, figsize=(6, 6))

    COORDS_TO_KEEP = SNAKE_SIZE
    last_coords = np.zeros((COORDS_TO_KEEP, 2))

    def animate(i):
        ax.set_title(f"Orbit ({i*SKIP_FRAMES_STEP/(NUM_OF_FRAMES_TO_USE-SKIP_FIRST_N_POINTS)*100:.3f}%)")
        # update last coords like a fifo queue
        last_coords[1:, :] = last_coords[:-1, :]
        last_coords[0, 0] = data1[i]
        last_coords[0, 1] = data2[i]
        if SCATTER_FLAG:
            ax.scatter(data1[i], data2[i], color='k', s=1)
        elif PLOT_SNAKE_LIKE_FLAG:
            ax.clear()
            # plot lines connecting last 5 points
            for j in range(1, COORDS_TO_KEEP):
                ax.plot([last_coords[j-1, 0], last_coords[j, 0]], [last_coords[j-1, 1], last_coords[j, 1]], color='k')
        else:
            # add the plot of the last two points
            ax.plot(last_coords[0:2, 0], last_coords[0:2, 1], color='k')

        ax.set_xlim(-x_range, x_range)
        ax.set_ylim(-y_range, y_range)
        ax.set_xlabel(x_title)
        ax.set_ylabel(y_title)


    ani = animation.FuncAnimation(fig, animate, frames=range(0, NUM_OF_FRAMES_TO_USE, SKIP_FRAMES_STEP), interval=ANIM_INTERVAL, repeat=False)

    plt.show()

# -----------------------------------------------------------------------------
