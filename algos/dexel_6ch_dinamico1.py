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
    Este archivo representa la salida real del sistema
    la salida son dos funciones transferencia, la out del sistema y la realimentada
    la realimentada se utiliza para el lazo pid, y la salida del pid es u_pid 'zoh'
    el pid revisa cada tanto y ejecuta los cambios
    funciones transfrencia, step y digital en arcchivos dexel_6ch_dinamico y dexel_6ch_modelo
    en principio simulo 100ms
"""

#Elementos de Hardware segun modelo de LTSpice
L = 180e-6
Rsense = 0.33
Rf = 10e3
Cf = 47e-9

#leds segun modelo
Vd = 23.3
Rd = 2.93

#Alimentacion del PWM
Vin = 35

#etapa de potencia
s = Symbol('s')
TY1 = Rsense / (s*L + Rd + Rsense)

#etapa de filtro
Tfiltro = 1 / (s*Cf*Rf + 1)

Iout = TY1 / Rsense
Iout_sim = Iout.simplify()

Isense = TY1 * Tfiltro
Isense_sim = Isense.simplify()

planta_out = sympy_to_lti(Iout_sim)
planta_real = sympy_to_lti(Isense_sim)

### Pruebo step con d=0.9
t_linear = np.linspace(0, 0.01, num=2000)
u = np.ones_like(t_linear)
u = u * (0.9*Vin-Vd)
t, y, x = lsim(planta_real, T=t_linear, U=u)
t, y1, x1 = lsim(planta_out, T=t_linear, U=u)

# fig, ax = plt.subplots()
# ax.set_title('Respuesta escalon')
# ax.set_ylabel('Vsense')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y, 'r-')
# ax.plot(t, y1, 'b-')

# plt.tight_layout()
# plt.show()

### Desde aca sistema Digital
### Convierto Forward Euler
Fsampling = 4800
Tsampling = 1 / Fsampling
num_d1, den_d1, td = cont2discrete((planta_real.num, planta_real.den), Tsampling, method='bilinear')

tfinal = 0.01
num = tfinal * Fsampling
t = np.linspace(0, tfinal, num=num)

b = num_d1
a = den_d1
vin = np.ones_like(t)
vin = vin * (0.9*Vin-Vd)
vout = np.zeros_like(t)

b = np.transpose(b)
# print ("b type")
# print (type(b))
# print ("shape")
# print (np.shape(b))
# print ("b params:")
# print (b[0])
# print ("a type")
# print (type(a))
# print ("shape")
# print (np.shape(a))
# print ("a params:")
# print (a[0])

for i, x in enumerate(vin):
    if i >= 2:
        vout[i] = b[0]*vin[i] + b[1]*vin[i-1] + b[2]*vin[i-2] - a[1]*vout[i-1] - a[2]*vout[i-2]

        
fig, ax = plt.subplots()
ax.set_title('Compara continuo con digital')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t_linear, y, 'b-')
ax.plot(t_linear, y1, 'r-')
ax.stem(t, vout, 'y-')
# ax.stem(t, y1, 'g-')

plt.tight_layout()
plt.show()
