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
#Ts = 0.0005        #fs = 2000Hz 40 muestras
Ts = 0.00025       #fs = 4000Hz 80 muestras

#vector lenght
#lenght = 313       #fs = 15686Hz 313 muestras
lenght = 80        #fs = 2000Hz 40 muestras
#lenght = 320        #fs = 4000Hz 80 muestras

#time vector
t = np.arange (0, lenght * Ts, Ts)

fig1, (ax1, ax2) = plt.subplots(2,1)     #API Matplotlib
#figure(1)
#clf()# clear the figure - "#" is the comment symbol in Python
#fig.clf()

freq = 50
#current vector
imax = 1

#theta_i = 1*pi/8
#theta_i = pi/2.0
theta_i = 0
it = imax * sin(2*pi*freq*t+theta_i)             #para probar cos phi
ax1.plot(t,it)
ax1.set_ylabel('Current')

#voltage vector
vmax = 312
theta_v = pi/8.0
#theta_v = theta_i
vt = 1 * sin(2*pi*freq*t+theta_v)

vt_rect1 = (vt - 0.1) * 1.11
vt_rect2 = np.where(vt_rect1 < 0, 0, vt_rect1)
vt_rect2 = vt_rect2 * vmax

ax2.plot(t,vt_rect2)
ax2.set_ylabel('Voltage')

plt.show()

power_instant = it * vt_rect2
pact = power_instant.sum()/lenght
pact_ajust = pact * 2.064

print ""
print " -Algoritmo Analogico-"
print "Pact: " + str('{:.2f}'.format(pact))
print "Pact_ajust: " + str('{:.2f}'.format(pact_ajust))

pact_teorico = (imax * vmax) / 2.0
pact_error = ((pact_teorico - pact) / pact_teorico) * 100
pact_error_ajust = ((pact_teorico - pact_ajust) / pact_teorico) * 100

print "Error Pact respecto de " + str(pact_teorico) + "W: " + str('{:.2f}'.format(pact_error)) + "%"
print "Error Pact_ajust respecto de " + str(pact_teorico) + "W: " + str('{:.2f}'.format(pact_error_ajust)) + "%"



"""
    desde aca para abajo Valores digitales por el ADC
"""

fig2, (ax3, ax4) = plt.subplots(2,1)     #API Matplotlib

#current vector
zero_current = 2048
itd = np.uint16(zero_current + it * 1024)

ax3.stem(t,itd)
ax3.set_ylabel('Current')

#voltage vector
vt_rect3 = np.uint16((vt_rect2 / 360.0) * 4095)

ax4.stem(t,vt_rect3)
ax4.set_ylabel('Voltage')

plt.show()


print ""
print " -Algoritmo Digital-"

""" 
 Comienzo el algoritmo, determino primero Izero
"""

izerod = np.uint16(itd.sum() / lenght)
print "Izero = " + str(izerod)

iacd = np.int16(itd - izerod)
pactd = np.int32(iacd * vt_rect3)

pactd = pactd >> 8
pactd_2 = pactd.sum() / lenght

print "Patd = " + str(pactd_2)

fig3, (ax1, ax2) = plt.subplots(2,1)     #API Matplotlib

#current vector
ax1.stem(t,iacd)
ax1.set_ylabel('Current D')

#power vector
ax2.stem(t,pactd)
ax2.set_ylabel('Power D')

plt.show()

pactd_2_ajust = pactd_2 * 0.0453484
pactd_error_ajust = ((pact_teorico - pactd_2_ajust) / pact_teorico) * 100

print "Patd_ajust = " + str('{:.2f}'.format(pactd_2_ajust))
print "Error Pactd_ajust respecto de " + str(pact_teorico) + "W: " + str('{:.2f}'.format(pactd_error_ajust)) + "%"
