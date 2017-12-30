import sympy
from scipy import signal
from sympy import *

from pylab import *

import matplotlib.pyplot as plt

sympy.init_printing()

#desde el algoritmo hacia atras
#uk = uk-1 + kp ek + kd (ek - ek-1)
#uk = uk-1 + kp ek + kd ek - kd ek-1
#Uz/Ez = (b0 + b1 z-1) / (1 - z-1)
#b0 = kp + kd
#b1 = -kd
#a0 = 1
#a1 = -1

kp = 0.025
ki = 0.05
kd = 0.1

zeros = [100, 1000]
poles = [0, 10000]

gain = 1
b, a = signal.zpk2tf(zeros, poles, gain)

print ("b vale")
print (b)
print ("a vale")
print (a)
f=arange(0.1,100000,1)
w, h = signal.freqresp((zeros, poles, gain), f)
w, mag, phase = signal.bode((zeros, poles, gain))


plt.figure()
plt.semilogx(w, mag)    # Bode magnitude plot
plt.figure()
plt.semilogx(w, phase)  # Bode phase plot
plt.show()
