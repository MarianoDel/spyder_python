# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, bode, lsim, convolve, TransferFunction
from tc_udemm import sympy_to_lti, lti_to_sympy

"""
        Analog part of the MicroInverter Power Plant
	Current Filter sensor and Current Output power
"""


###################################################################
# Ecuacion Transferencia del filtro de salida y etapa de potencia #
# Elementos de Hardware del filtro de salida                      #
###################################################################

Lout = 1.45e-3
Cout = 0.44e-6
Rload = 220

Vpwm = 250

#resultados de la etapa de potencia
s = Symbol('s')
Zout = Rload / (s**2 * 2*Lout*Cout*Rload + s*2*Lout + Rload)
Plant_out = Vpwm * Zout

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
kp = 0.05
ki = 300
kd = 0

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

fpwm = 50
Vsense = 200.0
t = np.linspace(0, 0.1, num=2000)
Vref = Vsense * np.sin(2*np.pi*fpwm*t)
t, y, x = lsim(close_loop, Vref, t)

fig.clear()
fig, ax = plt.subplots()
ax.set_title('Respuesta filtro antes y despues')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, y)
ax.plot(t, Vref)

plt.tight_layout()
plt.show()


#####################################################
# Mas pruebas temporales, OJO ESTO ES MUY MUY LENTO #
#####################################################
error = np.zeros_like(t)
Vout = np.zeros_like(t)

for i in range(len(t)):
    error[i] = Vref[i] - Vout[i]
    t1, Vout[i:], x = lsim(open_loop, error[i:], t[i:])
    conv = convolve(error[i:], 
    

fig.clear()
fig, ax = plt.subplots()
ax.set_title('Respuesta filtro antes y despues')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, Vout, 'r')
ax.plot(t, Vref, 'g')
ax.plot(t, error, 'y')


plt.tight_layout()
plt.show()

# #####################################
# # Desde aca hago pruebas temporales #
# #####################################
# fpwm = 50
# t = np.linspace(0, 0.1, num=2000)
# Vin = Vpwm * np.sin(2*np.pi*fpwm*t)
# t, y, x = lsim(planta, Vin, t)

# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Respuesta filtro antes y despues')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
# ax.plot(t, Vin)

# plt.tight_layout()
# plt.show()

# w, mag, phase = dbode(open_loop, n = 10000)

# fig, (ax1, ax2) = plt.subplots(2,1)

# ax1.semilogx(w/(2*np.pi), mag, 'b')
# ax1.set_title('Digital OpenLoop')
# ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
# ax1.set_xlabel('Frequency [Hz]')
# ax1.set_ylim([-20, 40])

# ax2.semilogx(w/(2*np.pi), phase, 'r')
# ax2.set_ylabel('Phase', color='r')
# ax2.set_xlabel('Frequency [Hz]')

# plt.tight_layout()
# plt.show()

# #####################################
# # Desde aca hago pruebas temporales #
# #####################################
# fpwm = 50
# t = np.linspace(0, 0.1, num=2000)
# Vin = Vpwm * np.sin(2*np.pi*fpwm*t)
# t, y, x = lsim(planta, Vin, t)

# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Respuesta filtro antes y despues')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
# ax.plot(t, Vin)

# plt.tight_layout()
# plt.show()


# ################################################
# # Elementos de Hardware que pueden ser moviles #
# # Caracteristica del filtro de sensado         #
# ################################################
# R1 = 1800
# C1 = 56e-9
# Ri = 1000
# Rf = 8200
# Cf = 5.6e-9
# Zi = Ri
# Zf = Rf / (1 + s*Cf*Rf)
# Aop = 1 + Zf/Zi
# Z1 = 1 / (1 + s*C1*R1)

# Filter_out = Z1 * Aop
# Filter_out_sim = Filter_out.simplify()

# print ('Filter_out: ')
# print (Filter_out_sim)

# #####################################################
# # Desde aca utilizo ceros y polos que entrego sympy #
# #####################################################
# filter_sense = sympy_to_lti(Filter_out_sim)
# print ("filter con sympy:")
# print (filter_sense)

# # freq = np.arange(1, 1000000, 1)

# w, mag, phase = bode(filter_sense, freq)

# fig, (ax1, ax2) = plt.subplots(2,1)
# ax1.semilogx (w/6.28, mag, 'b-', linewidth="1")
# ax1.set_title('Magnitude')

# ax2.semilogx (w/6.28, phase, 'r-', linewidth="1")
# ax2.set_title('Phase')

# plt.tight_layout()
# plt.show()

# #####################################
# # Desde aca hago pruebas temporales #
# #####################################f
# # fpwm = 50
# # t = np.linspace(0, 0.1, num=2000)
# Vin = np.sin(2*np.pi*fpwm*t)
# t, y, x = lsim(filter_sense, Vin, t)

# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Respuesta filtro antes y despues')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
# ax.plot(t, Vin)

# plt.tight_layout()
# plt.show()


# #######################################
# # Convierto Planta y Filtro a Digital #
# # por Tustin                          #
# #######################################
# Fsampling = 12000
# Tsampling = 1 / Fsampling
# planta_dig_tustin_n, planta_dig_tustin_d, td = cont2discrete((planta.num, planta.den), Tsampling, method='tustin')

# #normalizo con TransferFunction
# print ("Planta Digital:")
# planta_dig_tustin = TransferFunction(planta_dig_tustin_n, planta_dig_tustin_d, dt=td)
# print (planta_dig_tustin)

# filter_dig_tustin_n, filter_dig_tustin_d, td = cont2discrete((filter_sense.num, filter_sense.den), Tsampling, method='tustin')

# #normalizo con TransferFunction
# print ("Filter Digital:")
# filter_dig_tustin = TransferFunction(filter_dig_tustin_n, filter_dig_tustin_d, dt=td)
# print (filter_dig_tustin)

# ########################
# # Ecuacion PID Digital #
# ########################
# # kp = 3.96    #ziegler-nichols
# # ki = 3960
# # kd = 0.001
# ki_dig = ki / Fsampling
# kp_dig = kp - ki_dig / 2
# kd_dig = kd * Fsampling

# k1 = kp_dig + ki_dig + kd_dig
# k2 = -kp_dig - 2*kd_dig
# k3 = kd_dig

# #este es el pid
# b_pid = [k1, k2, k3]
# a_pid = [1, -1]
# print ("")
# print ("kp_dig: " + str(kp_dig) + " ki_dig: " + str(ki_dig) + " kd_dig: " + str(kd_dig))
# print ("")
# #este es custom controller
# # b_pid = [1.03925, -0.96075, 0]
# # a_pid = [1, -1]

# pid_dig = TransferFunction(b_pid, a_pid, dt=td)
# print ("PID Digital:")
# print (pid_dig)

# w, mag, phase = dbode(pid_dig, n = 10000)

# fig, (ax1, ax2) = plt.subplots(2,1)

# ax1.semilogx(w/(2*np.pi), mag, 'b')
# ax1.set_title('PID Digital')
# ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
# ax1.set_xlabel('Frequency [Hz]')

# ax2.semilogx(w/(2*np.pi), phase, 'r')
# ax2.set_ylabel('Phase', color='r')
# ax2.set_xlabel('Frequency [Hz]')

# plt.tight_layout()
# plt.show()

# ############################################
# # Multiplico Transferencias para OpenLoop  #
# ############################################
# c = lti_to_sympy(pid_dig)
# p = lti_to_sympy(filter_dig_tustin)

# ol = c * p

# open_loop = sympy_to_lti(ol)
# open_loop = TransferFunction(open_loop.num, open_loop.den, dt=td)   #normalizo

# w, mag, phase = dbode(open_loop, n = 10000)

# fig, (ax1, ax2) = plt.subplots(2,1)

# ax1.semilogx(w/(2*np.pi), mag, 'b')
# ax1.set_title('Digital OpenLoop')
# ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
# ax1.set_xlabel('Frequency [Hz]')
# ax1.set_ylim([-20, 40])

# ax2.semilogx(w/(2*np.pi), phase, 'r')
# ax2.set_ylabel('Phase', color='r')
# ax2.set_xlabel('Frequency [Hz]')

# plt.tight_layout()
# plt.show()


# #########################################
# # Realimento punto a punto con setpoint #
# #########################################

# # Respuesta escalon de la planta punto a punto
# print('td:')
# print (td)
# t = np.arange(0, 0.1, td)

# # Tustin
# b_planta = np.transpose(filter_dig_tustin_n)
# a_planta = np.transpose(filter_dig_tustin_d)

# vin_plant = np.zeros(t.size)
# vout_plant = np.zeros(t.size)


# ############################################
# # Armo la senial que quiero en el SETPOINT #
# ############################################
# muestras = np.size(t)
# Vmax = 1

# s_sen = np.zeros(muestras)

# for i in range(np.size(s_sen)):
#     s_sen[i] = np.sin(2*np.pi*i/muestras) * Vmax

# for i in range (np.size(s_sen)):
#     if s_sen[i] < 0:
#         s_sen[i] = 0

# # vin_setpoint = np.ones(t.size) * Vmax
# vin_setpoint = s_sen


# d = np.zeros(t.size)
# error = np.zeros(t.size)


# max_d_pwm = 1.0    
# for i in range(2, len(vin_plant)):
#     ###################################################
#     # primero calculo el error, siempre punto a punto #
#     ###################################################
#     error[i] = vin_setpoint[i] - vout_plant[i-1]

#     ###################
#     # aplico lazo PID #
#     ###################
#     d[i] = b_pid[0] * error[i] + b_pid[1] * error[i-1] + b_pid[2] * error[i-2] - a_pid[1] * d[i-1]

#     ###########################################
#     # ajusto los maximos permitidos en el pwm #
#     ###########################################
#     if d[i] > max_d_pwm:
#         d[i] = max_d_pwm

#     if d[i] < -max_d_pwm:
#         d[i] = -max_d_pwm

#     ########################################
#     # aplico la transferencia de la planta #
#     ########################################
#     vin_plant[i] = d[i]
#     vout_plant[i] = b_planta[0]*vin_plant[i] \
#                     + b_planta[1]*vin_plant[i-1] \
#                     + b_planta[2]*vin_plant[i-2] \
#                     - a_planta[1]*vout_plant[i-1] \
#                     - a_planta[2]*vout_plant[i-2]
    
               
        
# fig, ax = plt.subplots()
# ax.set_title('Respuesta Realimentada punto a punto')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo en muestras')
# ax.grid()
# # ax.plot(t, d)
# ax.plot(t, error, 'g')
# ax.plot(t, vin_setpoint, 'y')
# ax.stem(t, vout_plant)
# plt.tight_layout()
# plt.show()

# #respuesta escalon
# # t = np.arange (0, 0.1, td)
# # tout, yout = dstep([planta_d1.num, planta_d1.den, td], t=t)
# # yout1 = np.transpose(yout)
# # yout0 = yout1[0]
# # yout = yout0[:tout.size]

# # plt.figure(1)
# # plt.clf()
# # plt.title('digital Step Response')

# # plt.stem(tout,yout)
# # plt.show()

# # ###otro mas ajustado, agrega zero en 1 y compensa ganancia
# # num_d2 = [0.091, 0.091]
# # den_d2 = [1, -0.8861]

# # #normalizo con lti
# # planta_d2 = lti(num_d2, den_d2)

# # print ('Numerador Digital sys 2')
# # print (planta_d2.num)
# # print ('Denominador Digital sys 2')
# # print (planta_d2.den)

# # tout, yout = dstep([planta_d2.num, planta_d2.den, td], t=t)
# # yout1 = np.transpose(yout)
# # yout0 = yout1[0]
# # yout = yout0[:tout.size]


# # plt.figure(1)
# # plt.clf()
# # plt.title('digital Step Response')

# # # print (yout)
# # plt.stem(tout,yout)
# # plt.show()

# # # en frecuencia segundo funcion transferencia dgital
# # # w, h = freqz(num_d, den_d,worN=np.logspace(0, 4, 1000))
# # w, h = freqz(planta_d1.num, planta_d1.den)
# # fig, (ax1, ax2) = plt.subplots(2,1)

# # ax1.semilogx(w/(2*pi)*Fsampling, 20 * np.log10(abs(h)), 'b')
# # ax1.set_title('Planta Euler')
# # ax1.set_ylabel('Amplitude P D1 [dB]', color='b')
# # ax1.set_xlabel('Frequency [Hz]')

# # angles = np.unwrap(np.angle(h))
# # ax2.semilogx (w/(2*pi)*Fsampling, angles*180/pi, 'r-', linewidth="1")
# # ax2.set_title('Angle')

# # plt.tight_layout()
# # plt.show(block=False)

# # # en frecuencia
# # # w, h = freqz(num_d, den_d,worN=np.logspace(0, 4, 1000))
# # w, h = freqz(planta_d2.num, planta_d2.den)
# # fig, (ax1, ax2) = plt.subplots(2,1)

# # ax1.semilogx(w/(2*pi)*Fsampling, 20 * np.log10(abs(h)), 'b')
# # ax1.set_title('Planta Euler + ajuste')
# # ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
# # ax1.set_xlabel('Frequency [Hz]')

# # angles = np.unwrap(np.angle(h))
# # ax2.semilogx (w/(2*pi)*Fsampling, angles*180/pi, 'r-', linewidth="1")
# # ax2.set_title('Angle')

# # plt.tight_layout()
# # plt.show()
