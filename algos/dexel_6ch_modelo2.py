# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf, tf2zpk, step2, lsim
from scipy.signal import cont2discrete, freqz, ZerosPolesGain, dstep, dlti, dlsim
from tc_udemm import sympy_to_lti, lti_to_sympy
from math import pi, sin


"""
    Primero comparo el lazo en s (tiempo continuo) con el modelo de LTSpice
    El modelo esta deducido a mano y toma en cuanta que Rsense es en realidad Zsense
    con Rf = 10k y Cf = 47nf
"""

#Elementos de Hardware segun modelo de LTSpice
L = 180e-6
Rsense = 0.33
Rf = 10e3
Cf = 47e-9

#leds segun modelo
Vd = 23.3
Rd = 2.93

#Alimentacion del PWM
Vpwm = 35

s = Symbol('s')
# #descomentar para ver la ecuacion sin evaluar
# Cf = Symbol('Cf')
# Rf = Symbol('Rf')
# Rsense = Symbol('Rsense')

# etapa de sensado
Rama2 = Rf + 1 /(s * Cf)
Rama1 = Rsense
Zsense = Rama2 * Rama1 / (Rama2 + Rama1)
Zsense_sim = Zsense.simplify()
print ("Zsense:")
print (Zsense_sim)

# #etapa de potencia
Zeq = Zsense / (s * L + Rd + Zsense)
print ("Zeq:")
print (Zeq)

ZVIsense = Zeq * 1 /(1 + s*Cf*Rf)
print ("ZVIsense:")
print (ZVIsense)

Vin = Vpwm - Vd    #saco aca factor d y que multiplique todo

planta = sympy_to_lti(0.9* Vin * ZVIsense)
print ('Numerador Planta Sympy: ' + str(planta.num))
print ('Denominador Planta Sympy: ' + str(planta.den))


# Bode - Open Loop -
freq = np.arange(1, 10000, 0.01)
w, mag, phase = bode(planta, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*pi), mag, 'b-', linewidth="1")
ax1.set_title('Magnitude')

ax2.semilogx (w/(2*pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

### Pruebo step con d=0.9, que ya esta incluido en la planta
t = np.linspace(0, 0.01, num=2000)
u = np.ones_like(t)
# u = u * (0.9*Vpwm-Vd)
# u = u * (0.9*Vin)
t, y, x = lsim(planta, T=t, U=u)
fig.clear()
fig, ax = plt.subplots()
ax.set_title('Respuesta escalon')
ax.set_ylabel('Vsense')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, y, 'r-')

plt.tight_layout()
plt.show()

