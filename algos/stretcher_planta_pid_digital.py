# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf, tf2zpk, step2, lsim, cont2discrete, dbode, TransferFunction
from scipy.signal import dstep, freqz
from tc_udemm import sympy_to_lti, lti_to_sympy
from math import pi, sin


"""
	Pruebo lazo PID solo con KP, los resultados del sistema los tomo punto a punto
	la ganancia del sistema la tomo de las mediciones.
	El lazo PID es backward-Euler
"""

#Elementos de Hardware que pueden ser moviles
#Caracteristica de la bobina
# L = 141e-3
# R = 24

#bobina stretcher
L = 420e-3
R = 46

#Alimentacion del PWM stretcher
Valim = 287
dmax = 0.95
Vpwm = Valim * dmax


#Elementos del Hardware mayormente fijos en la placa
Rsense = 0.33
Aopamp = 1 + 560 / 1000		#hace que la salida sea aprox. 2A/V

#resultados de la etapa de potencia
s = Symbol('s')
Iout = Vpwm / (s*L + R + Rsense)
Vsense = Iout * Rsense
Plant_out = Iout
Plant_sense = Vsense * Aopamp

Plant_out_sim = Plant_out.simplify()
Plant_sense_sim = Plant_sense.simplify()

print ('Plant_out: Iout: ')
print (Plant_out_sim)
print ('Plant_sense: Vsense in opamp: ')
print (Plant_sense_sim)

planta = sympy_to_lti(Plant_out_sim)
print ('Numerador Planta Sympy: ' + str(planta.num))
print ('Denominador Planta Sympy: ' + str(planta.den))

##############################
# Convierto Planta a Digital #
# por Forward Euler          #
# y por Tustin               #
##############################
Fsampling = 2000
Tsampling = 1 / Fsampling
planta_dig_tustin_n, planta_dig_tustin_d, td = cont2discrete((planta.num, planta.den), Tsampling, method='tustin')
planta_dig_euler_n, planta_dig_euler_d, td = cont2discrete((planta.num, planta.den), Tsampling, method='euler')

#normalizo con TransferFunction
print ("Planta en Digital")
planta_dig_tustin = TransferFunction(planta_dig_tustin_n, planta_dig_tustin_d, dt=td)
print (planta_dig_tustin)

planta_dig_euler = TransferFunction(planta_dig_euler_n, planta_dig_euler_d, dt=td)
print (planta_dig_euler)

#dbode devuelve w = pi / dt, 100 puntos
f_eval = np.arange(0, 0.5, 0.0001)    #de 0 a 1 en saltos de 0.01 de fsampling
   
w_tustin, mag_tustin, phase_tustin = dbode(planta_dig_tustin, n = 1000)
w_euler, mag_euler, phase_euler = dbode(planta_dig_euler, n = 1000)

fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w_tustin/(np.pi), mag_tustin, 'b')
ax1.semilogx(w_euler/(np.pi), mag_euler, 'r')
ax1.set_title('Planta digital Tustin blue; Euler red')
ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')
ax1.set_ylim([-20, 20])

ax2.semilogx(w_tustin/(np.pi), phase_tustin, 'b')
ax2.semilogx(w_euler/(np.pi), phase_euler, 'r')
ax2.set_ylabel('Phase', color='r')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()

########################
# Ecuacion PID Digital #
########################
kp = 1.2
ki = 0.0
kd = 0.0
ki_dig = ki / Fsampling
kp_dig = kp - ki_dig / 2
kd_dig = kd * Fsampling

k1 = kp_dig + ki_dig + kd_dig
k2 = -kp_dig - 2*kd_dig
k3 = kd_dig

b = [k1, k2, k3]
a = [1, -1]

pid_dig = TransferFunction(b, a, dt=td)
print (pid_dig)

w_pid, mag_pid, phase_pid = dbode(pid_dig, n = 1000)

fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w_pid/(np.pi), mag_pid, 'b')
ax1.set_title('PID Digital')
ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')
ax1.set_ylim([-20, 20])

ax2.semilogx(w_pid/(np.pi), phase_pid, 'b')
ax2.set_ylabel('Phase', color='r')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()

#Multiplico para OpenLoop
c = lti_to_sympy(pid_dig)
p = lti_to_sympy(planta_dig_tustin)

ol = c * p

open_loop = sympy_to_lti(ol)
open_loop = TransferFunction(open_loop.num, open_loop.den, dt=td)   #normalizo

w, mag, phase = dbode(open_loop, n = 1000)

fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(np.pi), mag, 'b')
ax1.set_title('Digital OpenLoop')
ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')
ax1.set_ylim([-20, 20])

ax2.semilogx(w/(np.pi), phase, 'r')
ax2.set_ylabel('Phase', color='r')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()

#################################
# Realimento Digital el Sistema #
#################################

cl = ol / (1 + ol)
close_loop = sympy_to_lti(cl)
close_loop = TransferFunction(close_loop.num, close_loop.den, dt=td)   #normalizo

print ("Ecuacion recursiva close loop:")
print (close_loop)
w, mag, phase = dbode(close_loop, n = 1000)

fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(np.pi), mag, 'b')
ax1.set_title('Digital CloseLoop')
ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')
ax1.set_ylim([-20, 20])

ax2.semilogx(w/(np.pi), phase, 'r')
ax2.set_ylabel('Phase', color='r')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()
