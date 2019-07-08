# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, bode, lsim, TransferFunction, step
from scipy.signal import cont2discrete, dbode
from tc_udemm import sympy_to_lti, lti_to_sympy

"""
        Only Voltage Control Loop
        Digital analisys of the
        Analog part of the D100W Power Plant
	PWM average model for the output (buck part) only
"""


#########################################################
# Transfer Function equation for the output voltage and #
# the output current.                                   #
#########################################################
Lout = 155e-6
Cout = 940e-6
# Rload = 2260    #esto es el consumo del circuito de salida, opamp, reguladores y precarga
Rload = 10    #esto es el consumo del circuito de salida, opamp, reguladores y precarga
Rsense = 0.11

# Vpwm = 63    #max output in the main transformer limited to 0.5 by the duty_cycle
Vpwm = 126    #max output in the main transformer no limited by the duty_cycle

#TF equation Voltage on output and voltage on Rsense
s = Symbol('s')
Z2 = 1/(s*Cout)
Z1 = s*Lout
Vth = Z2 / (Z1 + Z2)    #Vth without Vpwm
# Vth = Z2 / (Z1 + Z2)    #Vth without Vpwm
Rth = Z1 * Z2 / (Z1 + Z2)
Zout_load = (Rload / (Rth + Rload + Rsense)) * Vth

Plant_out = Vpwm * Zout_load

Plant_out_sim = Plant_out.simplify()

print ('Plant_out: ')
print (Plant_out_sim)

#####################################################
# Desde aca utilizo ceros y polos que entrego sympy #
#####################################################
planta = sympy_to_lti(Plant_out_sim)


############################
# Convierto Planta Digital #
# por Tustin               #
############################
Fsampling = 24000
Tsampling = 1 / Fsampling
planta_dig_tustin_n, planta_dig_tustin_d, td = cont2discrete((planta.num, planta.den), Tsampling, method='tustin')

#normalizo con TransferFunction
print ("Planta Digital:")
planta_dig_tustin = TransferFunction(planta_dig_tustin_n, planta_dig_tustin_d, dt=td)
print (planta_dig_tustin)

########################
# Ecuacion PID Digital #
########################
## Parametros analogicos del PID SOLO VOLTAGE MODE
kp = 0.01
ki = 1
kd = 0.000001
# kp = 0.1
# ki = 1
# kd = 0.0003

ki_dig = ki / Fsampling
kp_dig = kp - ki_dig / 2
kd_dig = kd * Fsampling

k1 = kp_dig + ki_dig + kd_dig
k2 = -kp_dig - 2*kd_dig
k3 = kd_dig

## este es el pid digital
b_pid = [k1, k2, k3]
a_pid = [1, -1]
print ("")
print ("kp_dig: " + str(kp_dig) + " ki_dig: " + str(ki_dig) + " kd_dig: " + str(kd_dig))
print ("")
#este es custom controller
# b_pid = [1.03925, -0.96075, 0]
# a_pid = [1, -1]

pid_dig = TransferFunction(b_pid, a_pid, dt=td)
print ("PID Digital:")
print (pid_dig)

w, mag, phase = dbode(pid_dig, n = 10000)

fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(2*np.pi), mag, 'b')
ax1.set_title('PID Digital')
ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')

ax2.semilogx(w/(2*np.pi), phase, 'r')
ax2.set_ylabel('Phase', color='r')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()

#######################################################
# Multiplico Transferencias para OpenLoop y CloseLoop #
#######################################################
c = lti_to_sympy(pid_dig)
p = lti_to_sympy(planta_dig_tustin)

ol = c * p
cl = ol/(1+ol)

open_loop = sympy_to_lti(ol)
open_loop = TransferFunction(open_loop.num, open_loop.den, dt=td)   #normalizo ol
close_loop = sympy_to_lti(cl)
close_loop = TransferFunction(close_loop.num, close_loop.den, dt=td)   #normalizo cl

w, mag_ol, phase_ol = dbode(open_loop, n = 10000)
w, mag_cl, phase_cl = dbode(close_loop, n = 10000)

fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(2*np.pi), mag_ol, 'b')
ax1.semilogx(w/(2*np.pi), mag_cl, 'y')
ax1.set_title('Digital OpenLoop Blue, CloseLoop Yellow')
ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')
ax1.set_ylim([-40, 40])

ax2.semilogx(w/(2*np.pi), phase_ol, 'b')
ax2.semilogx(w/(2*np.pi), phase_cl, 'y')
ax2.set_ylabel('Phase', color='r')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()

#########################################
# Realimento punto a punto con setpoint #
#########################################

# Respuesta escalon de la planta punto a punto
tiempo_de_simulacion = 0.2
print('td:')
print (td)
t = np.arange(0, tiempo_de_simulacion, td)

# Planta Digital por Tustin
b_planta = np.transpose(planta_dig_tustin_n)
a_planta = np.transpose(planta_dig_tustin_d)

vin_plant = np.zeros(t.size)
vout_plant = np.zeros(t.size)


############################################
# Armo la senial que quiero en el SETPOINT #
############################################
muestras = np.size(t)
Vout_sp = 35

vin_setpoint = np.ones(t.size) * Vout_sp

d = np.zeros(t.size)
error = np.zeros(t.size)

# ver si limite la accion del PWM
# max_d_pwm = 1.0    #limite la tension del PWM
max_d_pwm = 0.5    #no limite el PWM, funciona al maximo y limito el duty_cycle
for i in range(2, len(vin_plant)):
    ###################################################
    # primero calculo el error, siempre punto a punto #
    ###################################################
    error[i] = vin_setpoint[i] - vout_plant[i-1]

    ###################
    # aplico lazo PID #
    ###################
    d[i] = b_pid[0] * error[i] + b_pid[1] * error[i-1] + b_pid[2] * error[i-2] - a_pid[1] * d[i-1]

    ###########################################
    # ajusto los maximos permitidos en el pwm #
    ###########################################
    if d[i] > max_d_pwm:
        d[i] = max_d_pwm

    # if d[i] < -max_d_pwm:
    #     d[i] = -max_d_pwm

    if d[i] < 0:
        d[i] = 0
        
    ########################################
    # aplico la transferencia de la planta #
    ########################################
    vin_plant[i] = d[i]
    vout_plant[i] = b_planta[0]*vin_plant[i] \
                    + b_planta[1]*vin_plant[i-1] \
                    + b_planta[2]*vin_plant[i-2] \
                    - a_planta[1]*vout_plant[i-1] \
                    - a_planta[2]*vout_plant[i-2]
    
               
        
fig, ax = plt.subplots()
ax.set_title('Respuesta Realimentada punto a punto')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo en muestras')
ax.grid()
ax.plot(t, d, 'r')
ax.plot(t, error, 'g')
ax.plot(t, vin_setpoint, 'y')
ax.stem(t, vout_plant)
plt.tight_layout()
plt.show()

