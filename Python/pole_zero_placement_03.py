#DIGITAL FILTER BY
#POLE ZERO PLACEMENT
#IIR_filters_pole_zero_placement.pdf

import sympy
from scipy import signal
from sympy import *

from pylab import *

import matplotlib.pyplot as plt

z = Symbol('z')
#Zeros
fs = 44000.

freqz1 = 25.
freqz2 = 185.
r1 = 0.97
r2 = 0.97

num = (z - r1*e**(1.0j*freqz1/fs*2*pi)) * (z - r1*e**(1.0j*-freqz1/fs*2*pi)) * (z - r2*e**(1.0j*freqz2/fs*2*pi)) * (z - r2*e**(1.0j*-freqz2/fs*2*pi))

#Poles recordar complejo conjugado y oscilacion en 1 (circulo)
freqp1 = fs
den = (z - freqp1/fs)

D = num / den
print (D)

bdig = num.expand()
print (bdig)

adig = den.expand()
print (adig)

bdig = [1.0, - 3.87931070665274, + 5.64406277927486, - 3.65004344388956, + 0.88529281]
#adig = [1.0, -2., 1.]
adig = [1.0, -1.]

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

show() #may not be necessary depending on how your graphics thread is running.

figure(2)
clf()# clear the figure - "#" is the comment symbol in Python
plt.plot(w*fs/(2*pi),abs(h))
plt.show()

figure(3)
clf()# clear the figure - "#" is the comment symbol in Python
plt.semilogx(w*fs/(2*pi),20*np.log10(abs(h)))
plt.show()
