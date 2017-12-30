# -*- coding: utf-8 -*-
import numpy as np
from scipy import *
from pylab import *

import scipy.signal as signal

def mfreqz(b,a=1):

    w,h = signal.freqz(b,a)

    h_dB = 20 * log10 (abs(h))

    subplot(211)

    plot(w/max(w),h_dB)

    ylim(-150, 5)

    ylabel('Magnitude (db)') 

    xlabel(r'Normalized Frequency (x$\pi$rad/sample)')

    title(r'Frequency response')

    subplot(212)

    h_Phase = unwrap(arctan2(imag(h),real(h)))

    plot(w/max(w),h_Phase)

    ylabel('Phase (radians)') 

    xlabel(r'Normalized Frequency (x$\pi$rad/sample)')

    title(r'Phase response')

    subplots_adjust(hspace=0.5)

def impz(b,a=1):

        impulse = repeat(0.,50); impulse[0] =1.

        x = arange(0,50)

        response = signal.lfilter(b,a,impulse)

        subplot(211)

        stem(x, response)

        ylabel('Amplitude') 

        xlabel(r'n (samples)')

        title(r'Impulse response')

        subplot(212)

        step = cumsum(response)

        stem(x, step)

        ylabel('Amplitude') 

        xlabel(r'n (samples)')

        title(r'Step response')

        subplots_adjust(hspace=0.5)

#sample time
Ts = 0.0005

#vector lenght
lenght = 1024

#time vector
t = np.arange (0, lenght * Ts, Ts)

figure(1)
clf()# clear the figure - "#" is the comment symbol in Python


#current vector
subplot(211)
imax = 1
#it = imax * sin(2*pi*50*t-pi/40) + 0.25    #para analizar errores de muestreo
#it = imax * sin(2*pi*50*t+pi/40) + 0.25    #para analizar errores de muestreo
#it = imax * sin(2*pi*50*t) + 0.25         #para analizar corrimiento DC
#it = imax * sin(2*pi*50*t+pi/2)             #para probar cos phi
it = imax * sin(2*pi*50*t+5*pi/4)             #para probar cos phi
#it = imax * sin(2*pi*50*t+pi)             #para probar bidireccional
plot(t,it)
ylabel('Current')

#voltage vector
subplot(212)
vmax = 1
vt = vmax * sin(2*pi*50*t)
plot(t,vt)
ylabel('Voltage')

show()

figure(2)
clf()# clear the figure - "#" is the comment symbol in Python


#instant power
plot(t,it*vt)
ylabel('Instant Power')

show()

#filtro y señal filtrada
fstop = 5
fpass = 35
fsample = 1 / Ts
wp = fpass / fsample
ws = fstop / fsample
b,a = signal.iirdesign(wp, ws, gpass=1, gstop= 20, analog=False, ftype='butter')
response_i = signal.lfilter(b,a,it)
response_v = signal.lfilter(b,a,vt)
#figure(3)
#plot(t,response_i)
#ylabel('filtered output')
#show()

figure(4)
mfreqz(b,a)
show()
draw()

figure(5)
impz(b,a)
show()
draw()

ni2 = response_i**2
nv2 = response_v**2

#figure(5)
#plot(t,ni2)
#ylabel('ni^2')
#show()

ni3 = ni2.sum()/lenght
nv3 = nv2.sum()/lenght
irms = sqrt(ni3)
vrms = sqrt(nv3)

nresp = response_i * response_v
pact = nresp.sum()/lenght
s = irms * vrms
pf = pact / s
q = s*sqrt(1-pf**2)

print "irms: " + str(irms)
print "vrms: " + str(vrms)
print "Pact: " + str(pact)
print "S   : " + str(s)
print "PF  : " + str(pf)
print "Q   : " + str(q)