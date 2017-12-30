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
#it = imax * sin(2*pi*50*t) + 0.25
it = np.ones(1024)
it[20:39] = 0
it[60:79] = 0
it[100:119] = 0
it[140:159] = 0
it[180:199] = 0
it[220:239] = 0
it[260:279] = 0
it[300:319] = 0
it[340:359] = 0
it[380:399] = 0
it[420:439] = 0
it[460:479] = 0
it[500:519] = 0
it[540:559] = 0
it[580:599] = 0
it[620:639] = 0
it[660:679] = 0
it[700:719] = 0
it[740:759] = 0
it[780:799] = 0
it[820:839] = 0
it[860:879] = 0
it[900:919] = 0
it[940:959] = 0
it[980:999] = 0
it[1020:1023] = 0

plot(t,it)
ylabel('Current')

#voltage vector
subplot(212)
vmax = 1
vt = vmax * sin(2*pi*50*t)
plot(t,vt)
ylabel('Voltage')
show()

#instant power
figure(2)
clf()# clear the figure - "#" is the comment symbol in Python
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
response = signal.lfilter(b,a,it)
figure(3)
plot(t,response)
ylabel('filtered output')
show()

figure(4)
mfreqz(b,a)
show()

n2 = response**2
figure(5)
plot(t,n2)
ylabel('n^2')
show()

n3 = n2.sum()/lenght
irms = sqrt(n3)
print irms
