# -*- coding: utf-8 -*-
#usando libreria de control
#http://python-control.readthedocs.org/en/latest/index.html

import numpy as np
import control as ct
import matplotlib.pyplot as plt
#from scipy import signal

#los coeficientes los saco del sistema digital creado en Tfilter_sympy_02.py
#numd1
#array([ 0.        ,  1.48941694,  0.9759379 , -0.00450648])
#dend
#array([  1.00000000e+00,   9.21237959e-01,   5.39610404e-01, 1.33103857e-18])
b = [ 0.        ,  1.48941694,  0.9759379 , -0.00450648]
a = [  1.00000000e+00,   9.21237959e-01,   5.39610404e-01, 1.33103857e-18]
dt = 1.0/25000

plt.figure(1)
dsys1 = ct.tf(b, a, dt)
omega = np.arange(100, 3.1415 / dt, 1)
mag, phase, omega = ct.bode_plot(dsys1, omega)
plt.show()
plt.draw()

G1 = dsys1
G2 = 350./3.3
Gt = ct.series(G1, G2)

plt.figure(2)
mag, phase, omega = ct.bode_plot(Gt, omega)
plt.show()
plt.draw()

#LAZO PID
#desde el algoritmo hacia atras
#uk = uk-1 + k1 ek + k2 ek-1 + k3 ek-2
#Uz/Ez = (b0 + b1 z-1 + b2 z-2) / (1 - z-1)
#b0 = kp + kd + ki
#b1 = -kp - 2kd
#b2 = kd
#a0 = 1
#a1 = -1

fs = 25000
kp = 0.0015/fs
#kd = 0.80
kd = 0.0
ki = 58.0/fs
#ki = 0.0

bpid = [kp + kd + ki, -kp - 2*kd, kd]     #del spice
apid = [1, -1]

print ("bpid vale")
print (bpid)
print ("apid vale")
print (apid)

plt.figure(3)
Gpid = ct.tf(bpid, apid, dt)
mag, phase, omega = ct.bode_plot(Gpid, omega)
plt.show()
plt.draw()

#open loop
GH = ct.series(Gpid, Gt)

plt.figure(4)
mag, phase, omega = ct.bode_plot(GH, omega)
plt.show()
plt.draw()

#feedback
Gfeed = ct.feedback(GH, sys2=1, sign=-1)
plt.figure(5)
mag, phase, omega = ct.bode_plot(Gfeed, omega)
plt.show()
plt.draw()

plt.figure(6)
tin = np.arange(0.0, 0.005, 0.0001)
Tout, yout2 = ct.step_response(Gfeed, T=tin, X0=0.0, input=None, output=None, transpose=False,)
yout1 = np.transpose(yout2)
yout0 = yout1[0]
#yout = yout0[:50]
yout = yout0[:Tout.size]
plt.plot(Tout, yout)
plt.show()
plt.draw()
