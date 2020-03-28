# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

"""    
    Generate a signal made up of 10 Hz and 20 Hz
"""

fo = 15      #cutoff freq = 15Hz
wo = 2 * np.pi * fo
filter_order = 10

tiempo_de_simulacion = 1
t = np.linspace(0, tiempo_de_simulacion, num=2000)  # 1 second
sig = np.sin(2*np.pi*10*t) + np.sin(2*np.pi*20*t)
fig, ax1 = plt.subplots(1, 1)
ax1.plot(t, sig)
ax1.set_title('10 Hz and 20 Hz sinusoids')
ax1.axis([0, 1, -2, 2])
plt.show()

b, a = signal.butter(filter_order, wo, 'highpass', analog=True)
print ("b coefficient:")
print (b)
print ("a coefficient:")
print (a)

######################################
# Respuesta en frecuencia del filtro #
######################################
w, h = signal.freqs(b, a)
fig, ax1 = plt.subplots(1, 1)
ax1.semilogx(w/(2*np.pi), 20 * np.log10(abs(h)))
ax1.set_title('Butterworth filter frequency response')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude [dB]')
plt.margins(0, 0.1)
plt.grid(which='both', axis='both')
plt.axvline(fo, color='green') # cutoff frequency
ax1.axis([1, 100, -60, 10])
plt.show()

###############################################
# Senial de salida luego de aplicar el filtro #
###############################################
t, yout, xout = signal.lsim((b , a), sig, t)
fig, ax1 = plt.subplots(1, 1)
ax1.plot(t, yout)
ax1.set_title('Filtered on 15Hz HPF')
ax1.axis([0, 1, -2, 2])
plt.show()



