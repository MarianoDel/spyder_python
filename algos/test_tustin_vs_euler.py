# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf, tf2zpk, step2, lsim, cont2discrete, dbode, TransferFunction
from scipy.signal import dstep
from tc_udemm import sympy_to_lti
from math import pi, sin


"""
	Pruebo conversion tustin vs euler
        En frecuencia y en Respuesta escalon
"""

#bobina stretcher
L = 420e-3
R = 46

#Alimentacion del PWM stretcher
Valim = 287
dmax = 0.95
Vpwm = Valim * dmax


#Elementos del Hardware mayormente fijos en la placa
Rsense = 0.33
Aopamp = 1 + 560 / 1000		#hace que la salida sea aprox. 2A/V

#resultados de la etapa de potencia
s = Symbol('s')
Plant_out = 200 / (s * 420e-3 + 46)

Plant_out_sim = Plant_out.simplify()

print ('Plant_out:')
print (Plant_out_sim)

planta = sympy_to_lti(Plant_out_sim)
print ('Numerador Planta Sympy: ' + str(planta.num))
print ('Denominador Planta Sympy: ' + str(planta.den))

z, p, k = tf2zpk(planta.num, planta.den)
print ('Planta Ceros: ' + str(planta.zeros))
print ('Planta Polos: ' + str(planta.poles))
print ('Planta K: ' + str(k))
#
### Muestro la respuesta escalon de la planta a lazo abierto
#
t = np.linspace(0, 0.1, num=2000)
t, y = step2(planta, T=t)

fig, ax = plt.subplots()
ax.set_title('Respuesta de la Planta')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, y)
# ax.show()

### Desde aca utilizo ceros y polos que entrego sympy

freq = np.arange(1, 10000, 0.01)
w, mag, phase = bode(planta, freq)
# wc, magc, phasec = bode(control, freq)
# wo, mago, phaseo = bode(openl, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*pi), mag, 'b-', linewidth="1")
ax1.set_title('Magnitude de la planta')

ax2.semilogx (w/(2*pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

### Convierto Planta por Forward Euler
Fsampling = 2000
Tsampling = 1 / Fsampling
planta_dig_tustin_n, planta_dig_tustin_d, td = cont2discrete((planta.num, planta.den), Tsampling, method='tustin')
planta_dig_euler_n, planta_dig_euler_d, td = cont2discrete((planta.num, planta.den), Tsampling, method='euler')

#normalizo con TransferFunction
planta_dig_tustin = TransferFunction(planta_dig_tustin_n, planta_dig_tustin_d, dt=td)
print (planta_dig_tustin)

planta_dig_euler = TransferFunction(planta_dig_euler_n, planta_dig_euler_d, dt=td)
print (planta_dig_euler)

#dbode devuelve w = pi / dt, 100 puntos
f_eval = np.arange(0, 0.5, 0.0001)    #de 0 a 1 en saltos de 0.01 de fsampling
   
w_tustin, mag_tustin, phase_tustin = dbode(planta_dig_tustin, n = 1000)
w_euler, mag_euler, phase_euler = dbode(planta_dig_euler, n = 1000)

fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w_tustin/(np.pi), mag_tustin, 'b')
ax1.semilogx(w_euler/(np.pi), mag_euler, 'r')
ax1.set_title('Planta digital Tustin blue; Euler red')
ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')
ax1.set_ylim([-20, 20])

ax2.semilogx(w_tustin/(np.pi), phase_tustin, 'b')
ax2.semilogx(w_euler/(np.pi), phase_euler, 'r')
ax2.set_ylabel('Phase', color='r')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()

t = np.linspace(0, 0.1, num=2000)
tout_t, yout_t = dstep([planta_dig_tustin.num, planta_dig_tustin.den, td], t=t)
tout_e, yout_e = dstep([planta_dig_euler.num, planta_dig_euler.den, td], t=t)
yout1 = np.transpose(yout_t)
yout0 = yout1[0]
yout_t = yout0[:tout_t.size]
yout1 = np.transpose(yout_e)
yout0 = yout1[0]
yout_e = yout0[:tout_e.size]

fig, ax = plt.subplots()
ax.set_title('Respuesta escalon de la planta digital')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.stem(tout_t, yout_t)
ax.stem(tout_e, yout_e, '-.')

plt.tight_layout()
plt.show()
