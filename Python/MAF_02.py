import numpy as np
from scipy import *
from pylab import *

#sample time
Ts = 0.01

#freq vector
f = np.arange (0.001, 1 / Ts, 0.001)

#transfer function
#Hjw = 0.33 * (1 + exp(-2j*pi*f) + exp(-2*2j*pi*f))
Hjw = 1./8. * (1 + exp(-2j*pi*f) + exp(-2*2j*pi*f) + exp(-3*2j*pi*f) + exp(-4*2j*pi*f) + exp(-5*2j*pi*f) + exp(-6*2j*pi*f)+ exp(-7*2j*pi*f))

figure(1)
clf()# clear the figure - "#" is the comment symbol in Python
subplot(211)
title('Hjw')
f1 = f[0:500]
Hjw1 = Hjw[0:500]
semilogx(f1,20*log10(abs(Hjw1)))
ylabel('Mag in dB')

subplot(212)
plot(f1,abs(Hjw1))
ylabel('Mag in times')
xlabel('Freq (Hz)')

#semilogx(f,arctan2(imag(Hjw),real(Hjw))*180.0/pi)
#ylabel('Phase (deg.)')
#xlabel('Freq (Hz)')

show()