import sympy
from scipy import signal
from sympy import *

from pylab import *

import matplotlib.pyplot as plt

sympy.init_printing()

#s = Symbol('s')
f=arange(0.1,1000,0.1)
s=2.0j*pi*f

kp = 0.01
ki = 1.0
kd = 0.00001

D = (kp * s + ki + s**2 * kd) / s

figure(1)
clf()# clear the figure - "#" is the comment symbol in Python
subplot(211)
title('PID TF')
semilogx(f,20*log10(abs(D)))
ylabel('Mag. Ratio (dB)')

subplot(212)
semilogx(f,arctan2(imag(D),real(D))*180.0/pi)
ylabel('Phase (deg.)')
xlabel('Freq (Hz)')

show() #may not be necessary depending on how your graphics thread is running.

b = [kd, kp, ki]
a = [1.0, 0]
fs = 44000

bdig, adig = signal.bilinear(b, a, fs)
z, p, k = signal.tf2zpk(b, a)

print (z)
print (p)
print (k)

print ("bdig vale: ") 
print(bdig)
print ("adig vale: ")
print(adig)

#k1 = kp + ki + kd
#k2 = -kp - 2*kd
#k3 = kd
#bdig = [k1, k2*80, k3*400]
#adig = [1, -1]

#print ("ahora bdig vale: ") 
#print(bdig)
#print ("ahora adig vale: ")
#print(adig)

w, h = signal.freqz(bdig, adig)
#fs = 1000
figure(2)
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
