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
# por Tustin                 #
##############################
Fsampling = 2000
Tsampling = 1 / Fsampling
planta_dig_tustin_n, planta_dig_tustin_d, td = cont2discrete((planta.num, planta.den), Tsampling, method='tustin')

#normalizo con TransferFunction
print ("Planta en Digital")
planta_dig_tustin = TransferFunction(planta_dig_tustin_n, planta_dig_tustin_d, dt=td)
print (planta_dig_tustin)

########################
# Ecuacion PID Digital #
########################
# kp = 3.96    #ziegler-nichols
# ki = 3960
# kd = 0.001
kp = 2
ki = 1000
kd = 0
ki_dig = ki / Fsampling
kp_dig = kp - ki_dig / 2
kd_dig = kd * Fsampling

k1 = kp_dig + ki_dig + kd_dig
k2 = -kp_dig - 2*kd_dig
k3 = kd_dig

#este es el pid
b_pid = [k1, k2, k3]
a_pid = [1, -1]
#este es custom controller
# b_pid = [1.03925, -0.96075, 0]
# a_pid = [1, -1]

pid_dig = TransferFunction(b_pid, a_pid, dt=td)
print (pid_dig)

#########################################
# Realimento punto a punto con setpoint #
#########################################

# Respuesta escalon de la planta punto a punto
print('td:')
print (td)
t = np.arange(0, 0.1, td)

# Tustin
b_planta = [0.1579362, 0.1579362]
a_planta = [ 1.        , -0.94632544]

vin_plant = np.zeros(t.size)
vout_plant = np.zeros(t.size)


############################################
# Armo la senial que quiero en el SETPOINT #
############################################
muestras = 200
Vmax = 1

s_sen = np.zeros(muestras)

for i in range(np.size(s_sen)):
    s_sen[i] = np.sin(2*np.pi*i/muestras) * Vmax

for i in range (np.size(s_sen)):
    if s_sen[i] < 0:
        s_sen[i] = 0

# vin_setpoint = np.ones(t.size) * Vmax
vin_setpoint = s_sen


d = np.zeros(t.size)
error = np.zeros(t.size)

for i, x in enumerate(vin_plant):
    if i >= 2:
        ###################################################
        # primero calculo el error, siempre punto a punto #
        ###################################################
        error[i] = vin_setpoint[i] - vout_plant[i-1]

        ###################
        # aplico lazo PID #
        ###################
        d[i] = b_pid[0] * error[i] + b_pid[1] * error[i-1] + b_pid[2] * error[i-2] - a_pid[1] * d[i-1]
        #ajusto los maximos
        if d[i] > 0.95:
            d[i] = 0.95

        if d[i] < -0.95:
            d[i] = -0.95

        ########################################
        # aplico la transferencia de la planta #
        ########################################
        vin_plant[i] = d[i]
        vout_plant[i] = b_planta[0]*vin_plant[i] + b_planta[1]*vin_plant[i-1] - a_planta[1]*vout_plant[i-1]
    else:
        vout_plant[i] = 0
        vin_plant[i] = 0

        
fig, ax = plt.subplots()
ax.set_title('Respuesta Realimentada punto a punto')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo en muestras')
ax.grid()
# ax.plot(t, d)
ax.plot(t, error, 'g')
ax.plot(t, vin_setpoint, 'y')
ax.stem(t, vout_plant)
plt.tight_layout()
plt.show()
