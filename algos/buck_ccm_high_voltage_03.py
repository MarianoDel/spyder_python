# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, bode, lsim, TransferFunction, step
from scipy.signal import cont2discrete, dbode
from tc_udemm import sympy_to_lti, lti_to_sympy

"""
        Same as 01 but input voltage can be varing
        Only Voltage Control Loop, i get the sensed data throu a filter
        Digital analisys of the
        Analog part of the D100W Power Plant
	PWM average model for the output (buck part) only
"""

######################################
# Graficos que habilito para mostrar #
######################################
show_plant_step = 0
show_plant_bode = 0
show_analog_pid_bode = 0
show_open_loop_bode = 0

show_digital_pid_bode = 1
show_digital_open_loop_bode = 1
show_digital_close_loop_bode = 1
show_sample_by_sample_step = 1
show_digital_output_as_stem = 0
show_varing_vi = 1

############################################################################
# Utilizo resultados del PID analogico convertido a digital o digital puro #
############################################################################
use_analog_pid_as_digital = 1

##############################################################
# Si uso undersampling, poner el valor en undersampling_roof #
##############################################################
undersampling_roof = 0

##############################################################
# Si voy a poner una tension de alimentacion variable Vinput #
##############################################################
use_varing_vi = 1

#########################################################
# Transfer Function equation for the output voltage and #
# the output current.                                   #
#########################################################
Lout = 250e-6
Cout = 200e-6
# Rload = 2260    #esto es el consumo del circuito de salida, opamp, reguladores y precarga
Rload = 27.5    #esto es la carga que produce el mismo fincionamiento que el LED 50V 2A
Rsense = 0.11

Duty = 0.48
Vpwm = 115    #max output in the main transformer no limited by the duty_cycle

#TF equation Voltage on output and voltage on Rsense
s = Symbol('s')
Z2 = 1/(s*Cout)
Z1 = s*Lout
Vth = Z2 / (Z1 + Z2)    #Vth without Vpwm
Rth = Z1 * Z2 / (Z1 + Z2)
Zout_load = (Rload / (Rth + Rload + Rsense)) * Vth

Plant_out = Vpwm * Zout_load
Plant_no_vi_out = Zout_load

Plant_out_sim = Plant_out.simplify()
Plant_no_vi_out_sim = Plant_no_vi_out.simplify()

print ('Plant_out: ')
print (Plant_out_sim)
print ('Plant_vi_out: ')
print (Plant_no_vi_out_sim)

#####################################################
# Desde aca utilizo ceros y polos que entrego sympy #
#####################################################
planta = sympy_to_lti(Plant_out_sim)
planta_no_vi = sympy_to_lti(Plant_no_vi_out_sim)

####################################################
# Respuesta Escalon de la planta al Duty Propuesto #
####################################################
if show_plant_step:
    t = np.linspace(0, 0.2, num=2000)
    u = np.ones(t.size) * Duty
    t, y, x = lsim(planta, T=t, U=u)
    
    fig, ax = plt.subplots()
    ax.set_title('Plant - Step Response')
    ax.set_ylabel('Vout')
    ax.set_xlabel('Tiempo [s]')
    ax.grid()
    ax.plot(t, y, 'b')
    
    plt.tight_layout()
    plt.show()


########################################
# Respuesta en Frecuencia de la Planta #
########################################
freq = np.arange(1, 100000, 1)

if show_plant_bode:
    w, mag, phase = bode(planta, freq)

    fig, (ax1, ax2) = plt.subplots(2,1)
    ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
    ax1.set_title('Plant Tf - Magnitude')

    ax2.semilogx (w/(2*np.pi), phase, 'b-', linewidth="1")
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
        o la ganancia es Kp??

        Conclusion:
        elijo Kp para la ganancia media, ej 0dB Kp = 1
        elijo primer cero, ej 15.9Hz, Ki = 100
        elijo segundo cero, ej 1590Hz, Kd = 0.0001
"""
#################
# PID analogico #
#################
kp = 0.01
ki = 10
zero_d_hz = 800
w_d = 2 * np.pi * zero_d_hz
kd = kp / w_d
# new_pole = 6.28 * 2630
new_pole = 0

if new_pole != 0:
    Pid_out = (kp + ki/s + s*kd) * (new_pole / (s + new_pole))
else:
    Pid_out = (kp + ki/s + s*kd)

Pid_out_sim = Pid_out.simplify()

print ('Pid_out: ')
print (Pid_out_sim)

##############################################
# Grafico de Bode con Polos y Ceros de sympy #
##############################################
pid = sympy_to_lti(Pid_out_sim)
print ("PID con sympy:")
print (pid)

if show_analog_pid_bode:
    # freq = np.arange(1, 1000000, 1)
    w, mag, phase = bode(pid, freq)

    fig, (ax1, ax2) = plt.subplots(2,1)
    ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
    ax1.set_title('PID Tf - Magnitude')

    ax2.semilogx (w/(2*np.pi), phase, 'b-', linewidth="1")
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
open_loop = TransferFunction(open_loop.num, open_loop.den)   #normalizo ol

if show_open_loop_bode:
    w, mag_ol, phase_ol = bode(open_loop, freq)

    fig, (ax1, ax2) = plt.subplots(2,1)
    ax1.semilogx(w/(2*np.pi), mag_ol, 'b')
    ax1.set_title('Analog OpenLoop')
    ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
    # ax1.set_ylim([-40, 40])

    ax2.semilogx(w/(2*np.pi), phase_ol, 'b')
    ax2.set_ylabel('Phase', color='b')
    ax2.set_xlabel('Frequency [Hz]')

    plt.tight_layout()
    plt.show()


############################
# Convierto Planta Digital #
# por Tustin               #
############################
Fsampling = 24000
Tsampling = 1 / Fsampling
planta_dig_tustin_n, planta_dig_tustin_d, td = cont2discrete((planta.num, planta.den), Tsampling, method='tustin')
planta_no_vi_dig_tustin_n, planta_no_vi_dig_tustin_d, td = cont2discrete((planta_no_vi.num, planta_no_vi.den), Tsampling, method='tustin')
    
#normalizo con TransferFunction
print ("Planta Digital:")
planta_dig_tustin = TransferFunction(planta_dig_tustin_n, planta_dig_tustin_d, dt=td)
print (planta_dig_tustin)

########################
# Ecuacion PID Digital #
########################
if use_analog_pid_as_digital:
    # kp = 1
    # ki = 1
    # kd = 0
    ki_dig = ki / Fsampling
    kp_dig = kp - ki_dig / 2
    kd_dig = kd * Fsampling
    
else:
    kp_dig = 1
    ki_dig = 1
    kd_dig = 0


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

if show_digital_pid_bode:
    w, mag, phase = dbode(pid_dig, n = 10000)

    fig, (ax1, ax2) = plt.subplots(2,1)

    ax1.semilogx(w/(2*np.pi), mag, 'y')
    ax1.set_title('PID Digital')
    ax1.set_ylabel('Amplitude P D2 [dB]')
    ax1.set_xlabel('Frequency [Hz]')
    
    ax2.semilogx(w/(2*np.pi), phase, 'y')
    ax2.set_ylabel('Phase')
    ax2.set_xlabel('Frequency [Hz]')
    
    plt.tight_layout()
    plt.show()

#######################################################
# Multiplico Transferencias Digitales para OpenLoop   #
#######################################################
c = lti_to_sympy(pid_dig)
p = lti_to_sympy(planta_dig_tustin)

ol = c * p
cl = ol/(1+ol)

open_loop = sympy_to_lti(ol)
open_loop = TransferFunction(open_loop.num, open_loop.den, dt=td)   #normalizo ol

close_loop = sympy_to_lti(cl)
close_loop = TransferFunction(close_loop.num, close_loop.den, dt=td)   #normalizo cl

if show_digital_open_loop_bode:
    w, mag_ol, phase_ol = dbode(open_loop, n = 10000)
    w, mag_cl, phase_cl = dbode(close_loop, n = 10000)

    fig, (ax1, ax2) = plt.subplots(2,1)

    ax1.semilogx(w/(2*np.pi), mag_ol, 'b')
    if show_digital_close_loop_bode:
        ax1.semilogx(w/(2*np.pi), mag_cl, 'y')
        ax1.set_title('Digital OpenLoop Blue CloseLoop Yellow')
    else:
        ax1.set_title('Digital OpenLoop')

    ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
    ax1.set_xlabel('Frequency [Hz]')
    ax1.set_ylim([-50, 50])
    
    ax2.semilogx(w/(2*np.pi), phase_ol, 'b')
    if show_digital_close_loop_bode:
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
if use_varing_vi:
    b_planta = np.transpose(planta_no_vi_dig_tustin_n)
    a_planta = np.transpose(planta_no_vi_dig_tustin_d)
else:
    b_planta = np.transpose(planta_dig_tustin_n)
    a_planta = np.transpose(planta_dig_tustin_d)
    

vin_plant = np.zeros(t.size)
vout_plant = np.zeros(t.size)


############################################
# Armo la senial que quiero en el SETPOINT #
############################################
muestras = np.size(t)
Vout_sp = 50

vin_setpoint = np.ones(t.size) * Vout_sp

d = np.zeros(t.size)
error = np.zeros(t.size)

# ver si limite la accion del PWM
# max_d_pwm = 1.0    #limite la tension del PWM
max_d_pwm = 0.85    #no limite el PWM, funciona al maximo y limito el duty_cycle
undersampling = 0

#######################################
# Armo la senial que quiero en Vinput #
#######################################
if use_varing_vi:
    f_mains = 100
    vinput = Vpwm / 40 * np.sin(2* np.pi * f_mains * t) + Vpwm
else:
    vinput = np.ones(t.size) 
    
for i in range(2, len(vin_plant)):
    ###################################################
    # primero calculo el error, siempre punto a punto #
    ###################################################
    if undersampling == 0:
        error[i] = vin_setpoint[i] - vout_plant[i-1]

        #############################################################
        # aplico lazo PID y ajusto los maximo y minimos que permito #
        #############################################################
        d[i] = b_pid[0] * error[i] + b_pid[1] * error[i-1] + b_pid[2] * error[i-2] - a_pid[1] * d[i-1]

        if d[i] > max_d_pwm:
            d[i] = max_d_pwm

        if d[i] < 0:
            d[i] = 0

        undersampling = undersampling_roof
    else:
        d[i] = d[i-1]
        if undersampling_roof:
            undersampling -= 1

        
    ########################################
    # aplico la transferencia de la planta #
    ########################################
    vin_plant[i] = d[i] * vinput[i]
    vout_plant[i] = b_planta[0]*vin_plant[i] \
                    + b_planta[1]*vin_plant[i-1] \
                    + b_planta[2]*vin_plant[i-2] \
                    - a_planta[1]*vout_plant[i-1] \
                    - a_planta[2]*vout_plant[i-2]
    
               

if show_sample_by_sample_step:
    fig, ax = plt.subplots()
    ax.set_title('Respuesta Realimentada punto a punto')
    ax.set_ylabel('Corriente')
    ax.set_xlabel('Tiempo en muestras')
    ax.grid()
    ax.plot(t, d, 'm')
    ax.plot(t, error, 'g')
    ax.plot(t, vin_setpoint, 'y')
    if show_digital_output_as_stem:
        ax.stem(t, vout_plant)
    else:
        ax.plot(t, vout_plant, 'b')

    if show_varing_vi:
        ax.plot(t, vinput, 'k')
        
    plt.tight_layout()
    plt.show()

