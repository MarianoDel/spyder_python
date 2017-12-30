# -*- coding: utf-8 -*-
#usando libreria de control
#http://python-control.readthedocs.org/en/latest/index.html

import numpy as np
import control as ct
import matplotlib.pyplot as plt
#from scipy import signal

b = [0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125]
a = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#dt = 0.001
dt = 1.0

plt.figure(1)
dsys1 = ct.tf(b, a, dt)
omega = np.arange(0.1, 6.28, 6.28/1000)
mag, phase, omega = ct.bode_plot(dsys1, omega)
plt.show()


#x = np.array([[1, 2, 3], [4, 5, 6]], np.int32)


#T1 = np.arange(0.0, 0.02, 0.001)        #NOK con step_response
#T = np.ndarray((1,T1.size), dtype=float)
#T[0,:] = T1

T1 = np.arange(0.0, 0.02, 0.001)       #OK con step NOK con step_response
T = np.ndarray((T1.size,1), dtype=float)
T[:,0] = T1

#T1 = np.arange(0, 7, 1)
#T = np.ndarray((T1.size,1), dtype=float)
#T[:,0] = T1

#T, yout = ct.step_response(dsys1, T)
t, y = ct.step(dsys1, T)

plt.figure(2)
plt.clf()
plt.plot(omega, 20 * np.log10(abs(mag)), 'b')
plt.ylabel('Amplitude [dB]', color='b')
plt.xlabel('Frequency [rad/sample]')
plt.show()

#plt.figure(3)
#plt.clf()
#t2 = t[0, :]
#plt.plot(t2, y)
#plt.show()

plt.figure(3)
plt.clf()
t2 = T[0, :]
plt.plot(t2, y)
plt.show()
