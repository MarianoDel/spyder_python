import numpy as np
from scipy import *
from pylab import *

#sample time
Ts = 0.1

#freq vector
#f = np.arange (0, 1 / Ts, 0.001)
#f = np.arange (0.01, 1 / Ts, 0.01)
f = np.arange (0, 1, 0.001)

#transfer function
#Hjw = 0.33 * (1 + exp(-2j*pi*f) + exp(-2*2j*pi*f))
Hjw = 0.25 * (1 + exp(-1*2j*pi*f) + exp(-2*2j*pi*f) + exp(-3*2j*pi*f))
#Hjw = 1./8. * (1 + exp(-2j*pi*f) + exp(-2*2j*pi*f) + exp(-3*2j*pi*f) + exp(-4*2j*pi*f) + exp(-5*2j*pi*f) + exp(-6*2j*pi*f)+ exp(-7*2j*pi*f))

figure(1)
clf()# clear the figure - "#" is the comment symbol in Python
subplot(211)
title('Hjw')
plot(f, abs(Hjw))
#semilogx(f,20*log10(abs(Hjw)))
ylabel('Mag(dB)')

subplot(212)
semilogx(f,arctan2(imag(Hjw),real(Hjw))*180.0/pi)
ylabel('Phase (deg.)')
xlabel('Freq (Hz)')

show()