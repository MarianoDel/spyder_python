#DIGITAL FILTER BY
#POLE ZERO PLACEMENT
#IIR_filters_pole_zero_placement.pdf

import sympy
from scipy import signal
from sympy import *

from pylab import *

import matplotlib.pyplot as plt

bdig = [1.0, 0, -1.0]
adig = [1.0, 0, 0.877969]
fs = 500


w, h = signal.freqz(bdig, adig)
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
plt.plot(w*fs/(2*pi),abs(h))

show() #may not be necessary depending on how your graphics thread is running.
