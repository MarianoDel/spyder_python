import sympy
from scipy import signal
from sympy import *

from pylab import *

import matplotlib.pyplot as plt

sympy.init_printing()

#POLO EN ORIGEN Y ZERO (implementacion normal de PID) la otra implementacion

#desde el algoritmo hacia atras
#uk = uk-1 + kp ek + kd (ek - ek-1) + ki (ek + ek-1 )
#uk = uk-1 + kp ek + kd ek - kd ek-1 + ki ek + ki ek-1
#Uz/Ez = (b0 + b1 z-1) / (1 - z-1)
#b0 = kp + kd + ki
#b1 = ki - kd
#a0 = 1
#a1 = -1

fs = 44000
kp = 0.01
kd = 0.40
ki = 0.05

b = [kp + kd + ki, ki - kd]
a = [1, -1]

print ("b vale")
print (b)
print ("a vale")
print (a)

w, h = signal.freqz(b, a)

fig = plt.figure()
plt.title('Digital filter frequency response')
ax1 = fig.add_subplot(111)

#plt.semilogx(w*fs/(2*pi), 20 * np.log10(abs(h)), 'b')
plt.plot(w / pi, 20 * np.log10(abs(h)), 'b')
#plt.plot(w / pi, abs(h), 'b')
plt.ylabel('Amplitude [dB]', color='b')
plt.xlabel('Frequency Hz')
plt.grid()
plt.show()

#ax2 = ax1.twinx()
#angles = np.unwrap(np.angle(h))
#plt.semilogx(w, angles, 'g')
#plt.ylabel('Angle (radians)', color='g')
#plt.grid()
#plt.axis('tight')
#plt.show()

