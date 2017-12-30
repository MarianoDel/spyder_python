# -*- coding: utf-8 -*-
from numpy import *
from matplotlib import pyplot as plt
from scipy.signal import lti, step, bode

from sympy import *

#sympy.init_printing()
s = Symbol('s')

H = 441 * (s+3)
Gp = 1 / (s**2 + s + 9)
T = (H * Gp) / (1 + (H*Gp))

OpenLoop = H*Gp
OpenLoop_sim = OpenLoop.simplify()

Tsim = T.simplify()  # Esta función me permite ver como queda acomodada la función transferencia de H*Gp

print (Tsim)
print (OpenLoop_sim)

den1 = [1,1,9] 
num = [441,1323]
den = [1,442,1332]

num_open = [441, 1323]
den_open = den1


planta = lti(1, den1)
control = lti(num, den)
openl = lti(num_open, den_open)

freq = arange(0.1, 1000, 0.01)

w, mag, phase = bode(planta, freq)
wc, magc, phasec = bode(control, freq)
wo, mago, phaseo = bode(openl, freq)

plt.figure(3)
plt.semilogx (w, mag, color="blue", linewidth="1")
plt.semilogx (wc, magc, color="green", linewidth="1")
plt.semilogx (wo, mago, color="red", linewidth="1")
plt.show(block=False)

plt.figure(4)
plt.semilogx (w, phase, color="blue", linewidth="1.1")
plt.semilogx (wc, phasec, color="green", linewidth="1.1")
plt.semilogx (wo, phaseo, color="red", linewidth="1.1")
plt.show()

tp, sp = step(planta)
tc, sc = step(control)

tc, sc = step(control, T = linspace(0, 0.05, 100))


plt.figure(1)
plt.plot(tp, sp,color='r')
plt.grid()
plt.title('Planta Sin Controlador')
plt.legend(('Respuesta al Escalon',), loc=1)
plt.ylabel('Amp')
plt.show(block=False)


plt.figure(2)
plt.plot(tc, sc)
plt.grid()
plt.title('Con Controlador y Realimentado')
plt.legend(('Respuesta al Escalon',), loc=1)
plt.ylabel('Amp')
plt.show(block=False)


