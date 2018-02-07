# -*- coding: utf-8 -*-
import numpy as np
from scipy import *
from pylab import *

import matplotlib.pyplot as plt
import scipy.signal as signal

""" 
        Medidor de Potencia Activa para placa Redonda V1.1 
        Toma el ADC desde el Trigger TIM3, cada 15686Hz, Ts = 63.75us
        En un ciclo tiene 313.72 muestras
"""

""" La medicion de tension es solo de medio ciclo senoidal """

""" 9.4ms activo 10.6ms inactivo """

#sample time
#Ts = 0.00006375    #fs = 15686Hz 313 muestras
Ts = 0.0005        #fs = 2000Hz 40 muestras
#Ts = 0.00025       #fs = 4000Hz 80 muestras

#vector lenght
#lenght = 313       #fs = 15686Hz 313 muestras
lenght = 40        #fs = 2000Hz 40 muestras
#lenght = 320        #fs = 4000Hz 80 muestras

#time vector
t = np.arange (0, lenght * Ts, Ts)

figure(1)
clf()# clear the figure - "#" is the comment symbol in Python

freq = 50
#current vector
subplot(211)
imax = 1
#theta_i = 1*pi/4
#theta_i = pi/2.0
theta_i = 0
it = imax * sin(2*pi*freq*t+theta_i)             #para probar cos phi
plot(t,it)
ylabel('Current')

#voltage vector
subplot(212)
vmax = 1
#theta_v = 5.5*pi/4
theta_v = 0.0
vt = vmax * sin(2*pi*freq*t+theta_v)
#vt_rect = np.where(vt < 0, 0, vt)
vt_rect1 = (vt - 0.1) * 1.11
vt_rect2 = np.where(vt_rect1 < 0, 0, vt_rect1)
plot(t,vt)
plot(t,vt_rect2, 'g')
#plot(t,vt_rect_m, 'r')
ylabel('Voltage')

show()

figure(2)
clf()# clear the figure - "#" is the comment symbol in Python


#instant power
power_instant = it * vt_rect2
plot(t,power_instant)
ylabel('Instant Power')

show()

#filtro y seniñal filtrada
#fstop = 5
#fpass = 35
#fsample = 1 / Ts
#wp = fpass / fsample
#ws = fstop / fsample
#b,a = signal.iirdesign(wp, ws, gpass=1, gstop= 20, analog=False, ftype='butter')
#response_i = signal.lfilter(b,a,it)
#response_v = signal.lfilter(b,a,vt)

#ni2 = response_i**2
#nv2 = response_v**2
ni2 = it**2
nv2 = vt**2

ni3 = ni2.sum()/lenght
nv3 = nv2.sum()/lenght

irms = sqrt(ni3)
vrms = sqrt(nv3)

#nresp = response_i * response_v
#pact = nresp.sum()/lenght

pact = power_instant.sum()/lenght
s = irms * vrms
pf = pact / s
q = s*sqrt(1-pf**2)

print "irms: " + str(irms)
print "vrms: " + str(vrms)
print "Pact: " + str(pact)
print "S   : " + str(s)
print "PF  : " + str(pf)
print "Q   : " + str(q)

pact_teorico = (imax * vmax) / 2.0
pact_error = ((pact_teorico - pact) / pact_teorico) * 100
print "Error Pact respecto de " + str(pact_teorico) + "W: " + str(pact_error) + "%"
figure(3)
clf()

