import sympy
from scipy import signal
from sympy import *

import matplotlib.pyplot as plt

sympy.init_printing()

s = Symbol('s')

kp = 0.01
ki = 1.0
kd = 0.00001
Tf = kd / 20
D = (kp * s + ki + (s**2 * kd) / (1 + s*Tf)) / s

print (D)
E = simplify(D)
print (E)

#Dnum = [kd, kp, ki]
#Dden = [0, 1., 0]   #agregar con ceros hasta llegar al indice num
Enum = [1.0005e-5, 0.0100005, 1.0]
Eden = [5.0e-7, 1.0, 0]

#D = signal.lti(Dnum, Dden)
E = signal.lti(Enum, Eden)

w, mag, phase = signal.bode(E)



plt.figure()
plt.semilogx(w/6.28, mag)    # Bode magnitude plot

plt.figure()
plt.semilogx(w/6.28, phase)  # Bode phase plot
plt.show()
