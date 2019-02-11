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
	Pruebo lazo PID solo con KP, los resultados del sistema los tomo punto a punto
	la ganancia del sistema la tomo de las mediciones.
	El lazo PID es backward-Euler
"""

#Elementos de Hardware que pueden ser moviles
#Caracteristica de la bobina
# L = 141e-3
# R = 24

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
Iout = Vpwm / (s*L + R + Rsense)
Vsense = Iout * Rsense
Plant_out = Iout
Plant_sense = Vsense * Aopamp

Plant_out_sim = Plant_out.simplify()
Plant_sense_sim = Plant_sense.simplify()

print ('Plant_out: Iout: ')
print (Plant_out_sim)
print ('Plant_sense: Vsense in opamp: ')
print (Plant_sense_sim)


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


##############################
# Convierto Planta a Digital #
# por Forward Euler          #
# y por Tustin               #
##############################
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

t = np.linspace(0, 0.1, num=200)
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

# Respuesta escalon de la planta punto a punto
print('td:')
print (td)
t = np.arange(0, 0.1, td)

# Euler
b = [0.32458333]
a = [ 1.        , -0.94484524]

vin = np.ones(t.size)
vout_e = np.zeros(t.size)

for i, x in enumerate(vin):
    if i >= 1:
        vout_e[i] = b[0]*vin[i] - a[1]*vout_e[i-1]
    else:
        vout_e[i] = 0

# Tustin
b = [0.1579362, 0.1579362]
a = [ 1.        , -0.94632544]

vout_t = np.zeros(t.size)

for i, x in enumerate(vin):
    if i >= 1:
        vout_t[i] = b[0]*vin[i] + b[1]*vin[i-1] - a[1]*vout_t[i-1]
    else:
        vout_t[i] = 0
        
fig, ax = plt.subplots()
ax.set_title('Respuesta de la Planta punto a punto')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo en muestras')
ax.grid()
ax.stem(t, vout_e)
ax.stem(t, vout_t)
plt.tight_layout()
plt.show()


#Multiplico para OpenLoop
# c = lti_to_sympy(controller_d)
# p = lti_to_sympy(planta_d2)

# ol = c * p

# open_loop = sympy_to_lti(ol)
# open_loop = TransferFunction(open_loop.num, open_loop.den, dt=td)   #normalizo

# w, mag, phase = dbode(open_loop, n = 1000)

# fig, (ax1, ax2) = plt.subplots(2,1)

# ax1.semilogx(w/(np.pi), mag, 'b')
# ax1.set_title('Digital OpenLoop')
# ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
# ax1.set_xlabel('Frequency [Hz]')

# ax2.semilogx(w/(np.pi), phase, 'r')
# ax2.set_ylabel('Phase', color='r')
# ax2.set_xlabel('Frequency [Hz]')

# plt.tight_layout()
# plt.show()
