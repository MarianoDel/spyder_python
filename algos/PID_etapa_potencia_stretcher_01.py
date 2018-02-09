# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf, tf2zpk, step2


"""
	Pruebo lazo PID solo con KP, los resultados del sistema los tomo punto a punto
	la ganancia del sistema la tomo de las mediciones.
	El lazo PID es backward-Euler
"""

#Elementos de Hardware que pueden ser moviles
#Caracteristica de la bobina
L = 141e-3
R = 24

#Alimentacion del PWM
Vpwm = 35

#Elementos del Hardware mayormente fijos en la placa
Rsense = 0.33
Aopamp = 1 + 2200 / 1000		#hace que la salida sea aprox. 1V/1A

#resultados de la etapa de potencia
s = Symbol('s')
Iout = Vpwm / (s*L + R + Rsense)
Vout = Iout * Rsense
Plant_out = Vout * Aopamp

Iout_sim = Iout.simplify()
Vout_sim = Vout.simplify()
Plant_out_sim = Plant_out.simplify()

print ('Iout: ')
print (Iout_sim)
print ('Vout: ')
print (Vout_sim)
print ('Plant_out: ')
print (Plant_out_sim)

### Desde aca utilizo ceros y polos que entrego sympy
num_planta = [0, 36.96]				#esto es b0 s1 y b1 s0
den_planta = [0.141, 24.33]			#esto es a0 s1 y a1 s0

planta = lti(num_planta, den_planta)

freq = np.arange(1, 10000, 0.01)

w, mag, phase = bode(planta, freq)
# wc, magc, phasec = bode(control, freq)
# wo, mago, phaseo = bode(openl, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/6.28, mag, 'b-', linewidth="1")
ax1.set_title('Magnitude')

ax2.semilogx (w/6.28, phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()


### Controlador por polos ceros y constante
poles = [-1000 + 0.0j]	#polo en w = 1000
# poles = [-4.440 + 4.440j, -4.440 - 4.440j, -1.083 + 0.0j]
# zeros = [100 + 0.0j, 0.0 + 0.0j, 0.0 + 0.0]
zeros = [-100 + 0.0j]	#zero en w = 100
k = 200
controller = zpk2tf(zeros, poles, k)

w, mag, phase = bode(controller, freq)

fig.clear()
fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/6.28, mag, 'b-', linewidth="1")
ax1.set_title('Magnitude')

ax2.semilogx (w/6.28, phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

from tc_udemm import multiplico_sistemas, sumo_sistemas, realimento
open_loop = multiplico_sistemas(controller, planta)

w, mag, phase = bode(open_loop, freq)

fig.clear()
fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/6.28, mag, 'b-', linewidth="1")
ax1.set_title('Magnitude')

ax2.semilogx (w/6.28, phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

closed_loop = realimento(open_loop, ([1],[1]))
w, mag, phase = bode(closed_loop, freq)

# print ('Open Loop')
# print (open_loop)
# print ('Closed Loop')
# print (closed_loop)
# print (tf2zpk(*closed_loop))

fig.clear()
fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/6.28, mag, 'b-', linewidth="1")
ax1.set_title('Magnitude')

ax2.semilogx (w/6.28, phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()


### Desde aca hago pruebas temporales
t = np.linspace(0, 0.01, num=2000)
t, y = step2(closed_loop, T=t)

fig.clear()
fig, ax = plt.subplots()
ax.set_title('Respuesta escalon de la función transferencia luego del PID realimentado')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, y)


plt.tight_layout()
plt.show()
