# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf, tf2zpk, step2, lsim
from scipy.signal import cont2discrete, freqz, ZerosPolesGain, dstep, dlti, dlsim
from tc_udemm import sympy_to_lti, lti_to_sympy
from math import pi, sin


"""
    Este archivo representa la salida real del sistema, muestreada
    la salida es la misma funcion que se realimenta
    el pid revisa cada tanto y ejecuta los cambios
    funciones transfrencia, step y digital en arcchivos dexel_6ch_dinamico y dexel_6ch_modelo
    en principio simulo 100ms
"""

def ADCSample (analog_sample):
    return int(analog_sample / 3.3 * 4095)

#Elementos de Hardware segun modelo de LTSpice
L = 180e-6
Rsense = 0.33
Rf = 10e3
Cf = 47e-9

#leds segun modelo
Vd = 23.3
Rd = 2.93

#Alimentacion del PWM
Vpwm = 35
D_MAX_PERMITIDO = 900
D_FONDO_ESCALA = 1000

#etapa de potencia
s = Symbol('s')
TY1 = Rsense / (s*L + Rd + Rsense)

#etapa de filtro
Tfiltro = 1 / (s*Cf*Rf + 1)

Isense = TY1 * Tfiltro
Isense_sim = Isense.simplify()

planta_real = sympy_to_lti(Isense_sim)

### Pruebo step con d=0.9
t_linear = np.linspace(0, 0.01, num=2000)
u = np.ones_like(t_linear)
u = u * (0.9*Vpwm-Vd)
t, y, x = lsim(planta_real, T=t_linear, U=u)


### Desde aca sistema Digital
### Convierto bilinear
Fsampling = 4800
Tsampling = 1 / Fsampling
num_d1, den_d1, td = cont2discrete((planta_real.num, planta_real.den), Tsampling, method='bilinear')

tfinal = 0.01
# tfinal = 0.02
num = tfinal * Fsampling
t = np.linspace(0, tfinal, num=num)

b = num_d1
a = den_d1
vin = np.zeros_like(t)
vout = np.zeros_like(t)

b = np.transpose(b)
print ("shape b")
print (np.shape(b))
print (b)
print ("shape a")
print (np.shape(a))
print (a)

#ademas vin aplicado es Vpwm = Vin - Vd
d = 0
Kp = 1.0
Ki = 0.1
Kd = 3
sp = 820
d = 0
error = np.zeros_like(t)
error_z1 = 0
error_z2 = 0

K1 = Kp + Ki + Kd
K2 = Kp + Kd + Kd
K3 = Kd

for i, x in enumerate(vin):

    if i >= 2:
        error[i] = sp - ADCSample(vout[i-1])
        error_z1 = error[i - 1]
        error_z2 = error[i - 2]
        d = d + error[i] * K1 - error_z1 * K2 + error_z2 * K3
        if (d > D_MAX_PERMITIDO):
            d = D_MAX_PERMITIDO
        elif (d < 0):
            d = 0

        #ademas vin aplicado es Vpwm = Vin - Vd
        Vaplicada = Vpwm * d / D_FONDO_ESCALA - Vd
        if (Vaplicada < 0):
            vin[i] = 0
        else:
            vin[i] = Vaplicada

        vout[i] = b[0]*vin[i] + b[1]*vin[i-1] + b[2]*vin[i-2] - a[1]*vout[i-1] - a[2]*vout[i-2]
        
    elif i == 1:
        error[i] = sp - ADCSample(vout[i-1])
        error_z1 = error[i - 1]

        d = d + error[i] * K1 - error_z1 * K2
        if (d > D_MAX_PERMITIDO):
            d = D_MAX_PERMITIDO
        elif (d < 0):
            d = 0

        #ademas vin aplicado es Vpwm = Vin - Vd
        Vaplicada = Vpwm * d / D_FONDO_ESCALA - Vd
        if (Vaplicada < 0):
            vin[i] = 0
        else:
            vin[i] = Vaplicada

        vout[i] = b[0]*vin[i] + b[1]*vin[i-1] - a[1]*vout[i-1]
        
    else:
        vout[i] = b[0]*vin[i]

        

        
fig, ax = plt.subplots()
ax.set_title('Salida digital open - loop')
ax.set_ylabel('Tension de sensado de Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t_linear, y, 'b-')
# ax.plot(t_linear, y1, 'r-')
# ax.stem(t, vin, 'y-')
ax.stem(t, vout, 'c-')
# ax.plot(t, error, 'g-')
# ax.stem(t, error, 'g-')

plt.tight_layout()
plt.show()

fig, ax = plt.subplots()
ax.set_title('Muestreos ADC Error y d')
ax.set_ylabel('Tension de sensado de Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
# ax.plot(t_linear, y, 'b-')
# ax.plot(t_linear, y1, 'r-')
adc_vout = np.zeros_like(vout)
for i, x in enumerate(vout):
    adc_vout[i] = ADCSample(vout[i])

ax.stem(t, adc_vout, 'c-')
ax.stem(t, error, 'y-')
# ax.stem(t, error, 'g-')

plt.tight_layout()
plt.show()
