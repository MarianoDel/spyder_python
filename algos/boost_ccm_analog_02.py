# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
# import math
from scipy.signal import lti, bode, lsim, TransferFunction, step, step2
from scipy.signal import cont2discrete, dbode
from tc_udemm import sympy_to_lti, lti_to_sympy

"""
        Boost Voltage-Mode, ecuaciones segun:
        http://www.ti.com/lit/an/slva633/slva633.pdf
        Este a diferencia del Buck es un modelo de baja senial
        entonces debo estimar el Duty al que va a estar sometido
"""


#########################################################
# Transfer Function equation for the output voltage and #
# the output current.                                   #
#########################################################
L = 215.7e-6
Cout = 940e-6
RCout = 40e-3
Rload = 24
Vi = 12
Duty = 0.67

# Equivalent units
Leq = L /((1-Duty)**2)
w0 = 1 /(np.sqrt( Leq * Cout ))
Q = Rload /(Leq * w0)

# Sense probe
R1 = 1800
R2 = 22e3
alpha = R2/(R1+R2)
# alpha = 1
         

print ('Equivalent Params:')
print ('with a duty of: %.2f Leq: %f w0: %f Q: %f' %(Duty, Leq, w0, Q))

#TF equation Voltage and Lon output and voltage on Rsense
s = Symbol('s')

Plant_num = (1 + s * Cout * RCout) * (1 - s * Leq/Rload)
Plant_den = 1 + s /(w0*Q) + s**2/w0**2
Plant_out = alpha * (Vi/(1-Duty)**2) * (Plant_num/Plant_den)

Plant_out_sim = Plant_out.simplify()
print ('Plant_out: ')
print (Plant_out_sim)

#####################################################
# Desde aca utilizo ceros y polos que entrego sympy #
#####################################################
planta = sympy_to_lti(Plant_out_sim)
print ("planta con sympy:")
print (planta)


########################################
# Respuesta en Frecuencia de la Planta #
########################################
freq = np.arange(1, 100000, 1)
w, mag, phase = bode(planta, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
ax1.set_title('Plant Tf - Magnitude')

ax2.semilogx (w/(2*np.pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

#########################
# Controlador analogico #
#########################
fz1 = 80
fz2 = 180
fp1 = 8
fp2 = 1500
fp3 = 3000
wz1 = fz1 * 2 * np.pi
wz2 = fz2 * 2 * np.pi
wp1 = fp1 * 2 * np.pi
wp2 = fp2 * 2 * np.pi
wp3 = fp3 * 2 * np.pi

num_controller = (s + wz1) * (s + wz2)
den_controller = (s + wp1) * (s + wp2) * (s + wp3)
gain_controller = wp1 * wp2 * wp3 / (wz1 * wz2)
Controller_out = gain_controller * num_controller / den_controller
Controller_out_sim = Controller_out.simplify()
print ('Controller_out: ')
print (Controller_out_sim)

##############################################
# Grafico de Bode con Polos y Ceros de sympy #
##############################################
controller = sympy_to_lti(Controller_out_sim)
print ("Controller con sympy:")
print (controller)

freq = np.arange(1, 1000000, 1)

w, mag, phase = bode(controller, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
ax1.set_title('PID Tf - Magnitude')

ax2.semilogx (w/(2*np.pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

#######################################################
# Multiplico Transferencias para OpenLoop y CloseLoop #
#######################################################
c = lti_to_sympy(controller)
p = lti_to_sympy(planta)

ol = c * p
cl = ol/(1+ol)

open_loop = sympy_to_lti(ol)
open_loop = TransferFunction(open_loop.num, open_loop.den)   #normalizo ol
close_loop = sympy_to_lti(cl)
close_loop = TransferFunction(close_loop.num, close_loop.den)   #normalizo cl

w, mag_ol, phase_ol = bode(open_loop, freq)
w, mag_cl, phase_cl = bode(close_loop, freq)


fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx(w/(2*np.pi), mag_ol, 'b')
ax1.semilogx(w/(2*np.pi), mag_cl, 'y')
ax1.set_title('Analog OpenLoop Blue, CloseLoop Yellow')
ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')
ax1.set_ylim([-40, 40])

ax2.semilogx(w/(2*np.pi), phase_ol, 'b')
ax2.semilogx(w/(2*np.pi), phase_cl, 'y')
ax2.set_ylabel('Phase', color='r')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()

######################################
# Realimento y veo Respuesta escalon #
######################################
t = np.linspace(0, 0.2, num=2000)
t, y = step2(close_loop, T=t)

fig.clear()
fig, ax = plt.subplots()
ax.set_title('Respuesta escalon Close Loop')
ax.set_ylabel('Vout')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, y)

plt.tight_layout()
plt.show()

