# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf, tf2zpk
from tc_udemm import sympy_to_lti, lti_to_sympy

#Para PID el polo es siempre en 0
# algoritmo:
# (s^2 Kd + s Kp + Ki) / s
#
poles = [0]	#polo en w = 1000
# poles = [-4.440 + 4.440j, -4.440 - 4.440j, -1.083 + 0.0j]
# zeros = [100 + 0.0j, 0.0 + 0.0j, 0.0 + 0.0]
# zeros = [-100 + 0.0j]	#zero en w = 100
# zeros = [-100, -1000]
zeros = [-100]
k = 10
b, a = zpk2tf(zeros, poles, k)

f_eval = np.arange(0.1, 1000, 0.5)
w, mag, phase = bode((b, a), w=f_eval*2*np.pi)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
ax1.set_title('Magnitude')

ax2.semilogx (w/(2*np.pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

#PID con kp ki kd
k = 10
z1 = -100
z2 = -1000        #ojo si no tengo segundo zero cambia la ecuacion!!!

kp = k * (-z2-z1)
ki = k * z2 * z1
kd = k

s = Symbol('s')
pid_poly = (s**2 * kd + s * kp + ki) / s
print (pid_poly)

pid_tf = sympy_to_lti (pid_poly)

w, mag, phase = bode((pid_tf.num, pid_tf.den), w=f_eval*2*np.pi)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
ax1.set_title('Magnitude')

ax2.semilogx (w/(2*np.pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

### desde Kp Ki Kd a zpk ###
z, p, k = tf2zpk(pid_tf.num, pid_tf.den)

print ("zeros: ", str(z))
print ("poles: ", str(p))
print ("constant: ", str(k))


