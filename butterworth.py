# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

"""    
    Generate a signal made up of 10 Hz and 20 Hz, sampled at 1 kHz
"""

fs = 1000    #sampled = 1KHz
fo = 15      #cutoff freq = 15Hz
filter_order = 10

t = np.linspace(0, 1, fs, False)  # 1 second, sampled 1KHz
sig = np.sin(2*np.pi*10*t) + np.sin(2*np.pi*20*t)
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(t, sig)
ax1.set_title('10 Hz and 20 Hz sinusoids')
ax1.axis([0, 1, -2, 2])

sos = signal.butter(filter_order, fo, 'hp', fs=fs, output='sos')
filtered_sos = signal.sosfilt(sos, sig)
ax2.plot(t, filtered_sos)
ax2.set_title('After 15 Hz high-pass filter')
ax2.axis([0, 1, -2, 2])
ax2.set_xlabel('Time [seconds]')
plt.tight_layout()
plt.show()

b, a = signal.butter(filter_order, fo, 'hp', fs=fs)
print ("b coefficient:")
print (b)
print ("a coefficient:")
print (a)

w, h = signal.freqz(b, a, fs=fs)
fig = plt.semilogx(w, 20 * np.log10(abs(h)))
plt.title('Butterworth filter frequency response')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude [dB]')
plt.margins(0, 0.1)
plt.grid(which='both', axis='both')
plt.axvline(fo, color='green') # cutoff frequency
plt.show()

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(t, filtered_sos)
ax1.set_title('Filtered with sosfilt')
ax1.axis([0, 1, -2, 2])

filtered_ba = signal.lfilter(b, a, sig)
ax2.plot(t, filtered_ba)
ax2.set_title('Filtered with lfilter')
ax2.axis([0, 1, -2, 2])
ax2.set_xlabel('Time [seconds]')
plt.tight_layout()
plt.show()



