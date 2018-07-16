# -*- coding: utf-8 -*-
#usar python3
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import lti, step, bode, lsim
from tc_udemm import sympy_to_lti, lti_to_sympy
from sympy import *

s = Symbol('s')

### datos del vehiculo
# masa Kg
m = 1200
# friccion Ns/m y es proporcional a la velocidad
b = 75
F = 2250    #fuerza en N para desplazar 30m/s
#### sistema ####
# F = m . a
# m . d2(x)/d2t = F - b . v
# a = d(v)/dt
# m . d(v)/dt + b . v = F
# L[H(t)] = m . s Vs + b Vs = Fs
# Vs = Fs /(m . s + b)
Gpedal = F
Gp = 1 / (m * s + b)
# Gp = F / (m * s + b)
print (Gp.simplify())

#convierto sympy a lti
planta = sympy_to_lti(Gp * Gpedal)

### Pruebo step
t = np.linspace(0, 120, num=2000)
u = np.ones_like(t)
fopen_loop = u * F
tp, yopen_loop, x = lsim(planta, T=t, U=u)

Kp = 0.2
Ki = 0.05

Gc = Kp + Ki / s
controlador = sympy_to_lti(Gc)

System = Gc * Gpedal * Gp / (1 + Gc * Gpedal * Gp)
Force = Gc * Gpedal / (1 + Gc * Gpedal * Gp)
print (System.simplify())

realimentado = sympy_to_lti(System)
Foutput = sympy_to_lti(Force)

u = np.ones_like(t)
u = u * 30
tp, y, x = lsim(realimentado, T=t, U=u)
tp, fout, x = lsim(Foutput, T=t, U=u)

fig, ax = plt.subplots()
ax.set_title('Respuesta escalon velocidad')
ax.set_ylabel('V')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(tp, y, 'b-')
ax.plot(tp, yopen_loop, 'g-')

plt.tight_layout()
plt.show()

fig, ax = plt.subplots()
ax.set_title('Esfuerzo F en [N]')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(tp, fout, 'b-')
ax.plot(tp, fopen_loop, 'g-')

plt.tight_layout()
plt.show()

