# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

"""    
    Generate a signal made up of 1KHz 2KHz and 3KHz
"""

fo_l = 1800      #cutoff freq = 1500Hz
wo_l = 2 * np.pi * fo_l
fo_h = 2200      #cutoff freq = 15Hz
wo_h = 2 * np.pi * fo_h
filter_order = 3

tiempo_de_simulacion = 0.01
t = np.linspace(0, tiempo_de_simulacion, num=2000)  # 1 second
sig = np.sin(2*np.pi*1000*t) + np.sin(2*np.pi*2000*t) + np.sin(2*np.pi*3000*t)
fig, ax1 = plt.subplots(1, 1)
ax1.plot(t, sig)
ax1.set_title('1KHz 2KHz and 3KHz sinusoids')
ax1.axis([0, tiempo_de_simulacion, -3, 3])
plt.show()

b, a = signal.butter(filter_order, (wo_l, wo_h), 'bandpass', analog=True)
print ("b coefficient:")
print (b)
print ("a coefficient:")
print (a)

w, h = signal.freqs(b, a)
fig, ax1 = plt.subplots(1, 1)
ax1.semilogx(w/(2*np.pi), 20 * np.log10(abs(h)))
ax1.set_title('Butterworth filter frequency response')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude [dB]')
plt.margins(0, 0.1)
plt.grid(which='both', axis='both')
plt.axvline(fo_l, color='green') # cutoff frequency
plt.axvline(fo_h, color='green') # cutoff frequency
ax1.axis([100, 10000, -60, 10])
plt.show()


t, yout, xout = signal.lsim((b , a), sig, t)
fig, ax1 = plt.subplots(1, 1)
ax1.plot(t, yout)
ax1.set_title('BPF Filter 1.5KHz and 2.5KHz')
ax1.axis([0, tiempo_de_simulacion, -3, 3])
plt.show()



