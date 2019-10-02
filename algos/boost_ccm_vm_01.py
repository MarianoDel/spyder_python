# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, bode, lsim, TransferFunction, step
from scipy.signal import cont2discrete, dbode
from tc_udemm import sympy_to_lti, lti_to_sympy

"""
        Boost PFC Voltage-Mode
        La transferencia de energia no es directa, funciona en el ciclo apagado del PWM
        Es equivalente a tener un transformador de n: 1/(1-d); con d = duty cycle
        Refiero la tension de entrada y el inductor al circuito de salida para conocer vo(t)
        La salida del circuito queda igual a un buck, pero con Vi y L trasladada a la salida
"""


#########################################################
# Transfer Function equation for the output voltage and #
# the output current.                                   #
#########################################################
L = 100e-6
Cout = 100e-6
Rout = 10
Vi = 10

# L = 1.5e-3
# Cout = 23.5e-6
# Rout = 816
# Rsense = 0.33
# Vi = 311

#Voltage and L reflected to output
d = 0.4
Lref = L/((1-d)**2)
Vref = Vi/(1-d)

print ('Reflected Params:')
print ('Lref: %f Vref: %f with duty: %f' %(Lref, Vref, d))

#TF equation Voltage and Lon output and voltage on Rsense
s = Symbol('s')
Z2 = Rout/(1 + s*Cout*Rout)
Z1 = s*Lref
Zth = Z2 / (Z1 + Z2)

Plant_out = Vref * Zth

Plant_out_sim = Plant_out.simplify()
print ('Plant_out: ')
print (Plant_out_sim)

#####################################################
# Desde aca utilizo ceros y polos que entrego sympy #
#####################################################
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

"""    
        PID Analogico
        PID completo Tf = Kp + Ki/s + s Kd    Tf = 1/s * (s**2 Kd + s Kp + Ki)
        muy dificil analizar, basicamente polo en origen y dos ceros
        los dos ceros, segun los parametros elegidos, pueden llegar a ser complejos conjugados

        si fuese solo PI tengo Tf = 1/s * Kp * (s + Ki/Kp)
        esto es polo en origen w = 1; cero en w = Ki/Kp; ganancia Kp

        si fuese solo PD tengo Tf = Kd * (s + Kp/Kd)
        esto es cero en w = Kp/Kd y ganancia Kd

        Conclusion:
        elijo Kp para la ganancia media, ej 0dB Kp = 1
        elijo primer cero, ej 15.9Hz, Ki = 100
        elijo segundo cero, ej 1590Hz, Kd = 0.0001
"""
#################
# PID analogico #
#################
kp = 1
ki = 100
kd = 0.0003

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
# Parametros analogicos del PID SOLO VOLTAGE MODE
# kp = 1
# ki = 100
# kd = 0.0003
kp = 1
ki = 11304
kd = 0

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
tiempo_de_simulacion = 0.05
print('td:')
print (td)
t = np.arange(0, tiempo_de_simulacion, td)

# Planta Digital por Tustin
b_planta = np.transpose(planta_dig_tustin_n)
a_planta = np.transpose(planta_dig_tustin_d)

vin_plant = np.zeros(t.size)
vout_plant = np.zeros(t.size)
step_out = np.zeros(t.size)
step_in = np.zeros(t.size)


############################################
# Armo la senial que quiero en el SETPOINT #
############################################
muestras = np.size(t)
freq_vin = 50
freq_vin_corr = freq_vin*tiempo_de_simulacion

s_sen = np.zeros(muestras)

# for i in range(np.size(s_sen)):
#     s_sen[i] = np.sin(2*np.pi*freq_vin_corr*i/muestras) * 8
#     if s_sen[i] < 0:
#         s_sen[i] = -s_sen[i]

# vin_setpoint = s_sen
vin_setpoint = np.ones(t.size)

d = np.zeros(t.size)
error = np.zeros(t.size)
max_d_pwm = 1

for i in range(2, len(vin_plant)):
    ###################################################
    # primero calculo el error, siempre punto a punto #
    ###################################################
    error[i] = vin_setpoint[i] - vout_plant[i-1]

    #############################################################
    # aplico lazo PID y ajusto los maximo y minimos que permito #
    #############################################################
    d[i] = b_pid[0] * error[i] + b_pid[1] * error[i-1] + b_pid[2] * error[i-2] - a_pid[1] * d[i-1]

    if d[i] > max_d_pwm:
        d[i] = max_d_pwm

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

    step_in [i] = 1.0
    step_out[i] = b_planta[0]*step_in[i] \
                    + b_planta[1]*step_in[i-1] \
                    + b_planta[2]*step_in[i-2] \
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
ax.plot(t, step_out, 'c')
ax.stem(t, vout_plant)
plt.tight_layout()
plt.show()

# # def ma_circular (new_sample, v_samples, v_index):
# #     total_sum = 0
# #     v_samples[v_index] = new_sample
# #     if v_index < (len(v_samples) - 1):
# #         v_index = v_index + 1
# #     else:
# #         v_index = 0
# #     for i in range(len(v_samples)):
# #         total_sum = total_sum + v_samples[i]

# #     total_sum = total_sum / len(v_samples)

# #     return total_sum, v_index


# # vin_setpoint = np.ones(t.size) * Vout_sp
# # vout_plant = np.asarray(vin_setpoint)
# # v_filter = np.zeros(8)
# # index_filer = 0
# # for i in range(len(vout_plant)):
# #     (vout_plant[i], index_filer) = ma_circular(vin_setpoint[i], v_filter, index_filer)
               
        
# # fig, ax = plt.subplots()
# # ax.set_title('Respuesta Filtro MA8')
# # ax.set_ylabel('Salida')
# # ax.set_xlabel('Tiempo en muestras')
# # ax.grid()
# # ax.plot(t, vin_setpoint, 'y')
# # ax.stem(t, vout_plant)
# # plt.tight_layout()
# # plt.show()

