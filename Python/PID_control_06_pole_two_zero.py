import sympy
from scipy import signal
from sympy import *

from pylab import *

import matplotlib.pyplot as plt

sympy.init_printing()

#POLO EN ORIGEN Y DOS CEROS (implementacion teorica del PID)

#desde el algoritmo hacia atras
#uk = uk-1 + k1 ek + k2 ek-1 + k3 ek-2
#Uz/Ez = (b0 + b1 z-1 + b2 z-2) / (1 - z-1)
#b0 = kp + kd + ki
#b1 = -kp - 2kd
#b2 = kd
#a0 = 1
#a1 = -1

fs = 44000
kp = 0.54
kd = 80
#kd = 0.0
ki = 0.00012
#ki = 0.0

b = [kp + kd + ki, -kp - 2*kd, kd]     #del spice
#b = [0.4863, -0.865, 0.4]     #del programa
a = [1, -1]

print ("b vale")
print (b)
print ("a vale")
print (a)

w, h = signal.freqz(b, a, 100000)

fig = plt.figure()
plt.title('Digital filter frequency response')
ax1 = fig.add_subplot(111)

plt.semilogx(w*fs/pi, 20 * np.log10(abs(h)), 'b')
ax1.axis((1, 10000, -40, 20))
#plt.plot(w, 20 * np.log10(abs(h)), 'b')
#plt.plot(w / pi, abs(h), 'b')
plt.ylabel('Amplitude [dB]', color='b')
plt.xlabel('Frequency rad/sample')
plt.grid()
plt.show()

#ax2 = ax1.twinx()
#angles = np.unwrap(np.angle(h))
#plt.semilogx(w*fs/(2*pi), angles, 'g')
#plt.plot(w, angles, 'g')
#plt.ylabel('Angle (radians)', color='g')
#plt.grid()
#plt.axis('tight')
#plt.show()

