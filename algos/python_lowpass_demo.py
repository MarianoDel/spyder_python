from scipy import *
from pylab import *

f=arange(0.1,1000,0.1)
s=2.0j*pi*f

p=2.0*pi*10

tf=p/(s+p)

figure(1)
clf()# clear the figure - "#" is the comment symbol in Python
subplot(211)
title('tf=p/(s+p)')
semilogx(f,20*log10(abs(tf)))
ylabel('Mag. Ratio (dB)')

subplot(212)
semilogx(f,arctan2(imag(tf),real(tf))*180.0/pi)
ylabel('Phase (deg.)')
xlabel('Freq (Hz)')

show() #may not be necessary depending on how your graphics thread is running.
#You can turn plotting off with ioff() if you are going to make multiple changes to a plot and don't want to re-plot it several times.
#This could be used if you were going to plot something and then add labels and titles and a legend and your plot included many points.

savefig('fig1.png')
#savefig('fig1.eps')

