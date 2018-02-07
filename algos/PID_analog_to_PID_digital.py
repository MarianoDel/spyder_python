# -*- coding: utf-8 -*-
#DIGITAL FILTER BY
#POLE ZERO PLACEMENT
#IIR_filters_pole_zero_placement.pdf

#con los parámetros del PID recursivo intento determinar la respuesta en frecuencia

import sympy
from scipy import signal
from sympy import *

from pylab import *

import matplotlib.pyplot as plt

z = Symbol('z')
#Zeros
fs = 44000.

#kp_dig = 0.08
#ki_dig = 0.000305
#kd_dig = 2.1

#de analogico a digital
kp = 0.5
ki = 25.
kd = 0.0001
ki_dig = ki / fs
kp_dig = kp - ki_dig / 2
kd_dig = kd * fs

k1 = kp_dig + ki_dig + kd_dig
k2 = -kp_dig - 2*kd_dig
k3 = kd_dig

bdig = [k1, k2, k3]
adig = [1.0, -1.]

print (bdig)
print (adig)


#adig = [1.0, -2., 1.]


fss = int(fs)
w, h = signal.freqz(bdig, adig, worN = fss)

figure(1)
clf()# clear the figure - "#" is the comment symbol in Python
subplot(311)
title('digital TF')
semilogx(w,20*log10(abs(h)))
ylabel('Mag. Ratio (dB)')

subplot(312)
semilogx(w,arctan2(imag(h),real(h))*180.0/pi)
ylabel('Phase (deg.)')
xlabel('Freq (Hz)')

subplot(313)
#plt.plot(w*fs/(2*pi),abs(h))
plt.plot(w,abs(h))

plt.show() #may not be necessary depending on how your graphics thread is running.
plt.draw()

figure(2)
clf()# clear the figure - "#" is the comment symbol in Python
plt.semilogx(w*fs/(2*pi),20*np.log10(abs(h)))
plt.show()
plt.draw()
