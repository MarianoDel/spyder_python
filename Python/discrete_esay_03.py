# -*- coding: utf-8 -*-
#lo mismo que esay01 pero para Moving Average
#respuesta en frecuencia y análisis discreto

import numpy as np
from scipy import signal
#b = signal.firwin(80, 0.5, window=('kaiser', 8))
b = [0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125]
w, h = signal.freqz(b)

import matplotlib.pyplot as plt
fig = plt.figure()
plt.title('Digital filter frequency response')
ax1 = fig.add_subplot(111)

plt.plot(w, 20 * np.log10(abs(h)), 'b')
plt.ylabel('Amplitude [dB]', color='b')
plt.xlabel('Frequency [rad/sample]')

ax2 = ax1.twinx()
angles = np.unwrap(np.angle(h))
plt.plot(w, angles, 'g')
plt.ylabel('Angle (radians)', color='g')
plt.grid()
plt.axis('tight')
plt.show()

plt.figure(2)
plt.clf()
tf = (b, [1.0], 1.0)
t_in = [0.0, 1.0, 2.0, 3.0]
u = np.asarray([0.0, 0.0, 1.0, 1.0])
t_out, y = signal.dlsim(tf, u, t=t_in)
plt.plot(t_out, y)
plt.show()