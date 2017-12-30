# -*- coding: utf-8 -*-
import numpy as np
from scipy import *
from pylab import *

import scipy.signal as signal

#import matplotlib as plt
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



#velocidad de muestreo
fs = 2000
Ts = 1. / fs

#freq de la señal muestreada
fo = 50

#muestras en 1 periodo
k = fs / fo

#largo del vector de muestras
lenght = 10*k

#vector seno en unsigned short

print k
#time vector
t = np.arange (0, lenght * Ts, Ts)

#vin = 32767 * sin(2*pi*fo*t)
vin = 2047 * sin(2*pi*fo*t) + 2048   #max excursion en el ADC
#vin_int = int(vin[2])
vin_int = np.asanyarray(vin, 'int16') #paso de float a int16

figure(1)
clf()
stem(t, vin_int, 'b')
#plot(x, cos(x))
show() #muestra el grafico
draw()  #actualiza el grafico


#APLICACION DEL FILTRO CALCULADO
b = [0.9726465, -1.945293, 0.9726465]
a = [1., -1.94454465, 0.94604136]
figure(2)
clf()
impz(b,a)
show()
draw()

#algoritmo float
vout = np.ones(lenght)

for i, x in enumerate(vin):
    if i >= 2:
        vout[i] = b[0]*vin[i] + b[1]*vin[i-1] + b[2]*vin[i-2] - a[1]*vout[i-1] - a[2]*vout[i-2]

figure(3)
clf()
plot(t, vout, 'b')
plot(t, vin, 'g')
plot(t, np.ones(lenght)*2047, 'y--')
plot(t, np.ones(lenght)*-2047, 'y--')
show()
draw()