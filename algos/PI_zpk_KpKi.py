# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf, tf2zpk
from tc_udemm import sympy_to_lti, lti_to_sympy

#Para PI el polo es siempre en 0
# algoritmo:
# (s Kp + Ki) / s
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

#PI con kp ki
k = 10
z1 = -100

kp = k
ki = -k * z1

s = Symbol('s')
pid_poly = (s * kp + ki) / s
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

### desde Kp Ki a zpk ###
z, p, k = tf2zpk(pid_tf.num, pid_tf.den)

print ("zeros: ", str(z))
print ("poles: ", str(p))
print ("constant: ", str(k))

# Sampling (aunque en pid es velocidad a la que lo calculo)
kd = 0
ts = 0.0066
fs = 1 / ts

ki_dig = ki / fs
kp_dig = kp - ki_dig / 2
kd_dig = kd * fs

print ("kp_dig: ", kp_dig)
print ("ki_dig: ", ki_dig)
print ("kd_dig: ", kd_dig)


k1 = kp_dig + ki_dig + kd_dig
k2 = -kp_dig - 2*kd_dig
k3 = kd_dig

bdig = [k1, k2, k3]
adig = [1.0, -1.]

print (bdig)
print (adig)





