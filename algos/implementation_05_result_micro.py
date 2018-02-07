# -*- coding: utf-8 -*-
import numpy as np
from scipy import *
from pylab import *

import scipy.signal as signal

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
#vin = np.ones(lenght)*2047          #para probar el algoritmo del filtro respuesta escalon OFFSET 0 (con correccion de continua += 369)
#vin = np.zeros(lenght)          #para probar el algoritmo del filtro respuesta impulso
#vin[0] = 2047
#vin = 2047 * sin(2*pi*fo*t) + 1024  #max excursion en el ADC OFFSET -50 (con correccion de continua += 369)
vin = 2047 * sin(2*pi*fo*t) + 2048  #max excursion en el ADC OFFSET -50 (con correccion de continua += 369)
#vin = 2047 * sin(2*pi*fo*t) + 512  #max excursion en el ADC OFFSET 0 (con correccion de continua += 369)
#vin = 2047 * sin(2*pi*fo*t)  #max excursion en el ADC OFFSET 0 (con correccion de continua += 369)
#vin_int = int(vin[2])
vin_int = np.asanyarray(vin, 'int16') #paso de float a int16

figure(1)
clf()
stem(t, vin_int, 'b')
#plot(x, cos(x))
show() #muestra el grafico
draw()  #actualiza el grafico


#ALGORITMO FLOAT
#aplicacion filtro calculado
b = [0.9726465, -1.945293, 0.9726465]
a = [1., -1.94454465, 0.94604136]

vout = np.ones(lenght)

vout[0] = vin[0]
vout[1] = vin[1]
for i, x in enumerate(vin):
    if i >= 2:
        vout[i] = b[0]*vin[i] + b[1]*vin[i-1] + b[2]*vin[i-2] - a[1]*vout[i-1] - a[2]*vout[i-2]

figure(2)
clf()
plot(t, vout, 'b')
plot(t, vin, 'g')
plot(t, np.ones(lenght)*2047, 'y--')
plot(t, np.ones(lenght)*-2047, 'y--')

show()
draw()

#ALGORITMO FIXED POINT
#parametros punto fijo

#parametros con redondeo
#b0 = 31871
#b1 = -63741
#b2 = 31871
#a1 = -63717
#a2 = 30999

#parametros con redondeo trabajado buscando 0 con step response
#b0 = 31874
#b1 = -63733
#b2 = 31874
#a1 = -63717
#a2 = 30999

#parametros con redondeo trabajado buscando 0 con senoidal
b0 = 31870
b1 = -63740
b2 = 31870
a1 = -63717
a2 = 30999

#b0 = 63740
#b1 = -127480
#b2 = 63740
#a1 = -127434
#a2 = 61998

#parametros con truncado
#b0 = 31870
#b1 = -63741
#b2 = 31870
#a1 = -63716
#a2 = 30998

#parametros con truncado y ajustado 15bits
#b0 = 15935
#b1 = -31863
#b2 = 15935
#a1 = -31858
#a2 = 15499
#type(b0)

vout_int = np.ones(lenght, 'int16')     #cambio a int16
#vout_int = np.ones(lenght, 'float')
#vout_int.dtype

acc = 1

vout_int[0] = vin_int[0] - 2048
vout_int[1] = vin_int[1] - 2048
acc = 0
resto = 0
for i, x in enumerate(vin_int):
    if i >= 2:
        #vout_int[i] += b0*vin_int[i] + b1*vin_int[i-1] + b2*vin_int[i-2] - a1*vout_int[i-1] - a2*vout_int[i-2]
        acc += b0*vin_int[i] + b1*vin_int[i-1] + b2*vin_int[i-2] - a1*vout_int[i-1] - a2*vout_int[i-2]
        #ajuste overflow
        if i < (lenght - 1):
            #vout_int[i + 1] = vout_int[i] & 0x00007FFF  #guardo los bits que voy a truncar                                    
            resto = acc & 0x00007FFF  #guardo los bits que voy a truncar                                                
            #vout_int[i] = vout_int[i] / 32768
            vout_int[i] = acc / 32768
            acc = resto
    #        vout_int[i] = vout_int[i] / 16383
        else:
            vout_int[i] = vout_int[i] / 32768

#accumulator = 1.0

#for i, x in enumerate(vin_int):
#    if i >= 2:
#        accumulator = b0*vin_int[i]
#        accumulator += b1*vin_int[i-1]
#        accumulator += b2*vin_int[i-2]
#        accumulator -= a1*vout_int[i-1]
#        accumulator -= a2*vout_int[i-2]
    #        accumulator = accumulator / 32768
#        accumulator = accumulator / 65536
#        vout_int[i] = accumulator

#vout_int += 319
figure(3)
clf()
plot(t, vout_int, 'b')
plot(t, vin_int, 'g')
plot(t, np.ones(lenght)*2047, 'y--')
plot(t, np.ones(lenght)*-2047, 'y--')

show()
draw()

figure(4)
clf()
plot(t, vout - vout_int, 'c')
title('Integer Filter Error')
show()
draw()

