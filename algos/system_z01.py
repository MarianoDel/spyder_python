# -*- coding: utf-8 -*-
#usar python3

from scipy import signal
from sympy import *
import matplotlib.pyplot as plt
from plot_zplane2 import zplane
import numpy as np

z = Symbol('z')

#este es el sistema como umerador y denominador
Kpwm = 69 / 224
KP = 2.91
Kt = KP * Kpwm

num = Kt * z
den = z + Kt

system = num / den
# OpenLoop = H*Gp
# OpenLoop_sim = OpenLoop.simplify()

System_sim = system.simplify()  # Esta función me permite ver como queda acomodada la función transferencia de H*Gp

print (System_sim)
# print (OpenLoop_sim)

zeroes = [0]
poles = [-0.308035714285714]

#paso H(z) con potencias positivas b0 zn b1 zn-1, etc; a0 zn a1 zn-1
b = np.array([Kt, 0])
a = np.array([1, Kt * 0.022])
zplane(b,a)

#respuesta escalon
t = np.arange (0, 100, 1)
tout, yout = signal.dstep([b, a, 1], t=t)
yout1 = np.transpose(yout)
yout0 = yout1[0]
yout = yout0[:tout.size]


plt.figure(1)
plt.clf()
plt.title('digital Step Response')

# print (yout)
plt.stem(tout,yout)
plt.show()
