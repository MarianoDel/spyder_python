# -*- coding: utf-8 -*-
#usar python3
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import lti, step, bode, lsim
from tc_udemm import sympy_to_lti, lti_to_sympy
from sympy import *

s = Symbol('s')

a = 0.5 - 0.7j
b = 0.5 + 0.7j
H = 1 / ((s + a) * (s + b))

Tsim = H.simplify()  # Esta función me permite ver como queda acomodada la función transferencia de H*Gp

print (Tsim)

#convierto sympy a lti
planta = sympy_to_lti(H)

### Pruebo step
t = np.linspace(0, 20, num=2000)
# u = np.ones_like(t)
# tp, y, x = lsim(planta, T=t, U=u)
tp, y = step(planta, T=t)


# fig, ax = plt.subplots()
# ax.set_title('Respuesta escalon')
# ax.set_ylabel('V')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(tp, y, 'b-')

# plt.tight_layout()
# plt.show()

### comparo con la antitransformada por residuos
anti = 1.33 - 1.33 * np.exp(-0.5 * t) * np.cos(0.7 * t)
# anti = 1.33 - 1.38 * np.exp(-0.5 * t) * np.cos(0.7 * t)

fig, ax = plt.subplots()
ax.set_title('Respuesta escalon Antitransformada')
ax.set_ylabel('V')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(tp, y, 'b-')
ax.plot(t, anti, 'g-')

plt.tight_layout()
plt.show()
