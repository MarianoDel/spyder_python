# -*- coding: utf-8 -*-
import numpy as np
from scipy import *
from pylab import *

import matplotlib.pyplot as plt
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
#theta_i = 1*pi/4
theta_i = pi/2.0
#it = imax * sin(2*pi*50*t-pi/40) + 0.25    #para analizar errores de muestreo
#it = imax * sin(2*pi*50*t+pi/40) + 0.25    #para analizar errores de muestreo
#it = imax * sin(2*pi*50*t) + 0.25         #para analizar corrimiento DC
#it = imax * sin(2*pi*50*t+pi/2)             #para probar cos phi
#it = imax * sin(2*pi*50*t+5*pi/4)             #para probar cos phi
#it = imax * sin(2*pi*50*t+pi/4)             #para probar cos phi
it = imax * sin(2*pi*50*t+theta_i)             #para probar cos phi
#it = imax * sin(2*pi*50*t+pi)             #para probar bidireccional
plot(t,it)
ylabel('Current')

#voltage vector
subplot(212)
vmax = 1
#theta_v = 5.5*pi/4
theta_v = 0.0
vt = vmax * sin(2*pi*50*t+theta_v)
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

#figure(4)
#mfreqz(b,a)
#show()

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

figure(3)
clf()

if (pact > 0):
    rp = np.arange(0, pact, 0.01)
    theta_p = np.ones(rp.size)
    theta_p[:] = 0
else:
    rp = np.arange(0, (pact*-1), 0.01)
    theta_p = np.ones(rp.size)
    theta_p[:] = pi

if (q > 0):
    rq = np.arange(0, q, 0.01)
    theta_q = np.ones(rq.size)    
    theta_q[:] = pi/2
else:
    rq = np.arange(0, (q*-1), 0.01)
    theta_q = np.ones(rq.size)
    theta_q[:] = 3*pi/2

rs = np.arange(0, s, 0.01)
theta_s = np.ones(rs.size)
theta_s[:] = arccos(pf)

ri = np.arange(0, irms, 0.01)
theta_ii = np.ones(ri.size)
theta_ii[:] = theta_i

rv = np.arange(0, vrms, 0.01)
theta_vv = np.ones(rv.size)
theta_vv[:] = theta_v


ax = plt.subplot(111, polar=True)

ax.plot(theta_p, rp, color='r', linewidth=3)
ax.plot(theta_q, rq, color='b', linewidth=3)
ax.plot(theta_s, rs, color='g', linewidth=3)

ax.plot(theta_ii, ri, color='c')
ax.plot(theta_vv, rv, color='m')

ax.set_rmax(1.0)
ax.grid(True)

ax.set_title("AC Power Polar Plot", va='bottom')
ax.legend(('Real', 'Reactive', 'Apparent'), loc='upper left')

plt.show()
plt.draw()
