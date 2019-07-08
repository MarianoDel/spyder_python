# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, bode, lsim, TransferFunction, step
from tc_udemm import sympy_to_lti, lti_to_sympy

"""
        Analog part of the D100W Power Plant
	PWM average model for the output (buck part) only
"""


#########################################################
# Transfer Function equation for the output voltage and #
# the output current.                                   #
#########################################################
Lout = 155e-6
Cout = 940e-6
Rload = 2260
Rsense = 0.11

Vpwm = 63    #max output in the main transformer limited to 0.5 by the duty_cycle

#TF equation Voltage on output and voltage on Rsense
s = Symbol('s')
Z2 = 1/(s*Cout)
Z1 = s*Lout
Vth = Z2 / (Z1 + Z2)    #Vth without Vpwm
# Vth = Z2 / (Z1 + Z2)    #Vth without Vpwm
Rth = Z1 * Z2 / (Z1 + Z2)
Zout_load = (Rload / (Rth + Rload + Rsense)) * Vth
Zout_sense = (Rsense / (Rth + Rload + Rsense)) * Vth

Plant_out = Vpwm * Zout_load
Filter_out = Vpwm * Zout_sense

Plant_out_sim = Plant_out.simplify()

print ('Plant_out: ')
print (Plant_out_sim)

##############################################
# Grafico de Bode con Polos y Ceros de sympy #
##############################################
planta = sympy_to_lti(Plant_out_sim)
print ("planta con sympy:")
print (planta)

freq = np.arange(1, 1000000, 1)

w, mag, phase = bode(planta, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
ax1.set_title('Plant Tf - Magnitude')

ax2.semilogx (w/(2*np.pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

#################
# PID analogico #
#################
kp = 0.01
ki = 1
kd = 0.000001

# kp = 0.1
# ki = 1
# kd = 0.0003

Pid_out = kp + ki/s + s*kd
Pid_out_sim = Pid_out.simplify()

print ('Pid_out: ')
print (Pid_out_sim)

##############################################
# Grafico de Bode con Polos y Ceros de sympy #
##############################################
pid = sympy_to_lti(Pid_out_sim)
print ("PID con sympy:")
print (pid)

freq = np.arange(1, 1000000, 1)

w, mag, phase = bode(pid, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
ax1.set_title('PID Tf - Magnitude')

ax2.semilogx (w/(2*np.pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

###########################################
# Multiplico Transferencias para OpenLoop #
###########################################
c = lti_to_sympy(pid)
p = lti_to_sympy(planta)

ol = c * p

open_loop = sympy_to_lti(ol)
open_loop = TransferFunction(open_loop.num, open_loop.den)   #normalizo

w, mag, phase = bode(open_loop, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
ax1.set_title('Open Loop Tf - Magnitude')

ax2.semilogx (w/(2*np.pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

############################################
# Cierro el lazo y hago pruebas temporales #
############################################

cl = ol / (1 + ol)

close_loop = sympy_to_lti (cl)

t = np.linspace(0, 0.1, num=2000)
u = np.ones_like(t)
t, y, x = lsim(close_loop, u, t)

fig.clear()
fig, ax = plt.subplots()
ax.set_title('Respuesta filtro antes y despues')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, y)
ax.plot(t, u)

plt.tight_layout()
plt.show()

