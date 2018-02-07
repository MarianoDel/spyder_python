# -*- coding: utf-8 -*-
#usando libreria de control

import numpy as np
from scipy import signal

b = [0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125]
tf1 = (b, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.001)   #ver que dt coincida con el step de tiempo en discreto
#w, h = signal.freqz(b)
#w, h = signal.freqz(tf1)
w, h = signal.freqz((b, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.001))

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
tf = (b, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.001)   #ver que dt coincida con el step de tiempo en discreto
                                                            #ademas le genero un multilpe polo en origen para que no me diga que num > den
t_in = np.arange(0.0, 0.1, 0.001)
#t_in = np.arange(0.0, 4.0, 1.0)
#u = np.asarray([0.0, 0.0, 1.0, 1.0])
u = np.ones(np.size(t_in))
t_out, y = signal.dlsim(tf, u, t=t_in)
plt.plot(t_out, y, 'b')
plt.plot(t_out, u+0.1, 'g')
plt.show()
