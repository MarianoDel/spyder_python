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

kp = 1.5
ki = 0.0093
kd = 0.0

k1 = kp + ki + kd
k2 = -kp - 2*kd
k3 = kd

b = [k1, k2, k3]
a = [1, -1]

w, h = signal.freqz(b, a)

print ("b vale: ") 
print(b)
print ("a vale: ")
print(a)

fs = 44000
figure(1)
clf()# clear the figure - "#" is the comment symbol in Python
subplot(211)
title('digital PID TF')
semilogx(w*fs/pi,20*log10(abs(h)))
ylabel('Mag. Ratio (dB)')

subplot(212)
semilogx(w,arctan2(imag(h),real(h))*180.0/pi)
ylabel('Phase (deg.)')
xlabel('Freq (Hz)')

show() #may not be necessary depending on how your graphics thread is running.
