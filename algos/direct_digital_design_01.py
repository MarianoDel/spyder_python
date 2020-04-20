# -*- coding: utf-8 -*-
#usar python3
import numpy as np
from sympy import *
import matplotlib.pyplot as plt
from scipy.signal import lti, bode, lsim, dbode, zpk2tf, tf2zpk, step2, cont2discrete, dstep, freqz, freqs, dlti, TransferFunction
from tc_udemm import sympy_to_lti, lti_to_sympy, plot_s_plane, plot_argand

"""
        Control por método Direct Digital Design
        Arranco con sist analog y convierto a digital con ZOH
        
"""

##########################################################################
# Cuales son los Graficos que quiero mostrar por cuestiones de velocidad #
##########################################################################
Bode_Planta_Analog = False
Escalon_Planta_Analog = False
Polos_Ceros_Analog = False
Bode_Planta_Digital = True
Escalon_Planta_Digital = False
Polos_Ceros_Digital = False
Bode_Controlador_Digital = True
Bode_Sensor_OpenLoop_CloseLoop_Digital = True
Polos_Ceros_CloseLoop_Digital = True
Escalon_CloseLoop_Digital = False
Escalon_CloseLoop_Original_Digital = True

# TF
s = Symbol('s')
Gs = 1 /(s**2 + 6*s + 10)

print ('Plant_out:')
print (Gs)

#####################################################
# Desde aca utilizo ceros y polos que entrego sympy #
#####################################################
planta_TF = sympy_to_lti(Gs)
print ("planta transfer function:")
print (planta_TF)

#####################
# Bode de la Planta #
#####################
fmin = 0.01
fmax = 1000
wfreq = np.arange(2*np.pi*fmin, 2*np.pi*fmax, fmin)
w, mag_p, phase_p = bode(planta_TF, wfreq)

if Bode_Planta_Analog == True:
    fig, (ax1, ax2) = plt.subplots(2,1)
    ax1.semilogx (w/6.28, mag_p, 'b-', linewidth="1")
    ax1.set_title('Magnitude')

    ax2.semilogx (w/6.28, phase_p, 'b-', linewidth="1")
    ax2.set_title('Phase')

    plt.tight_layout()
    plt.show()

#####################
# Step de la Planta #
#####################
tiempo_de_simulacion = 3.0
t = np.linspace(0, tiempo_de_simulacion, num=2000)
u = np.ones_like(t)
t, y, x = lsim(planta_TF, u, t)

if Escalon_Planta_Analog == True:
    fig.clear()
    fig, ax = plt.subplots()
    ax.set_title('Respuesta de la Planta')
    ax.set_ylabel('Vout')
    ax.set_xlabel('Tiempo [s]')
    ax.grid()
    ax.plot(t, y, 'g-')
    ax.plot(t, u, color='orange')

    plt.tight_layout()
    plt.show()


if Polos_Ceros_Analog == True:
    plot_s_plane(planta_TF)



##################################################
# Convierto Planta por ZOH a una frecuencia alta #
# para que no afecte polos o ceros               #
##################################################
Fsampling = 20
Tsampling = 1/Fsampling
    
planta_dig_zoh_n, planta_dig_zoh_d, td = cont2discrete((planta_TF.num, planta_TF.den), Tsampling, method='zoh')

#normalizo con TransferFunction
print ("Planta Digital Zoh:")
planta_dig_zoh = TransferFunction(planta_dig_zoh_n, planta_dig_zoh_d, dt=td)
print (planta_dig_zoh)

w, mag_zoh, phase_zoh = dbode(planta_dig_zoh, n = 10000)

if Bode_Planta_Digital == True:
    fig, (ax1, ax2) = plt.subplots(2,1)

    ax1.semilogx(w/(2*np.pi), mag_zoh, 'y')    
    ax1.set_title('Digital ZOH')
    ax1.set_ylabel('Amplitude P D2 [dB]', color='g')
    ax1.set_xlabel('Frequency [Hz]')

    ax2.semilogx(w/(2*np.pi), phase_zoh, 'y')    
    ax2.set_ylabel('Phase', color='g')
    ax2.set_xlabel('Frequency [Hz]')

    plt.tight_layout()
    plt.show()

#############################################
# Verifico Respuesta Escalon Planta Digital #
#############################################
tiempo_de_simulacion = 3.0
t = np.linspace(0, tiempo_de_simulacion, num=(tiempo_de_simulacion*Fsampling))
tout, yout_zoh = dstep([planta_dig_zoh.num, planta_dig_zoh.den, td], t=t)
yout1 = np.transpose(yout_zoh)
yout0 = yout1[0]
yout_zoh = yout0[:tout.size]

if Escalon_Planta_Digital == True:
    fig, ax = plt.subplots()
    ax.set_title('Step Planta Digital ZOH Yellow')
    ax.set_ylabel('Salida Planta')
    ax.set_xlabel('Tiempo [s]')
    ax.grid()
    ax.plot(tout, yout_zoh, 'y')

    plt.tight_layout()
    plt.show()


if Polos_Ceros_Digital == True:
    plot_argand(planta_dig_zoh)


#############################
# Custom Digital Controller #
#############################
""" 
    Only for PI dig:
    w0 ~= ki_dig * Fsampling / kp_dig
    plateau gain ~= 20 log kp_dig
    w1 ~= kp_dig / (kd_dig * Fsampling) * 10    el 10 no se de donde sale???

"""
ki_dig = 0.31    # Fsampling = 20Hz
kp_dig = 10.0
kd_dig = 0.0

k1 = kp_dig + ki_dig + kd_dig
k2 = -kp_dig - 2*kd_dig
k3 = kd_dig

#este es el pid
b_pid = [k1, k2, k3]
a_pid = [1, -1]

controller_tf = TransferFunction(b_pid, a_pid, dt=td)

### ojo uso s en vez de z porque a sympy no le importa
# z_const = 57.4844
# z_num = s**2-1.705*s+0.7424
# z_den = (s-1)*(s-0.4817)
# z_const = 45.2072
# z_num = s**2-1.704*s+0.7421
# z_den = (s-1)*(s-0.6376)

# controller_sympy = z_const*z_num/z_den
# controller_tf = sympy_to_lti(controller_sympy)
# controller_tf = TransferFunction(controller_tf.num, controller_tf.den, dt=td)

print ("Digital Controller:")
print (controller_tf)

w, mag, phase = dbode(controller_tf, n = 10000)

if Bode_Controlador_Digital == True:
    fig, (ax1, ax2) = plt.subplots(2,1)
    ax1.semilogx(w/(2*np.pi), mag, 'c')
    ax1.set_title('Digital Controller')
    ax1.set_ylabel('Amplitude P D2 [dB]', color='c')
    ax1.set_xlabel('Frequency [Hz]')

    ax2.semilogx(w/(2*np.pi), phase, 'c')
    ax2.set_ylabel('Phase', color='c')
    ax2.set_xlabel('Frequency [Hz]')

    plt.tight_layout()
    plt.show()


if Bode_Sensor_OpenLoop_CloseLoop_Digital == True:
    contr_dig = lti_to_sympy(controller_tf)
    plant_dig = lti_to_sympy(planta_dig_zoh)
    
    ol_dig = plant_dig * contr_dig
    open_loop_dig = sympy_to_lti(ol_dig)
    close_loop_dig = sympy_to_lti(ol_dig/(1+ol_dig))
    
    #normalizo con TransferFunction
    open_loop_dig = TransferFunction(open_loop_dig.num, open_loop_dig.den, dt=td)    
    close_loop_dig = TransferFunction(close_loop_dig.num, close_loop_dig.den, dt=td)

    w, mag_ol, phase_ol = dbode(open_loop_dig, n = 10000)
    w, mag_cl, phase_cl = dbode(close_loop_dig, n = 10000)
    
    fig, (ax1, ax2) = plt.subplots(2,1)
    ax1.semilogx(w/(2*np.pi), mag_ol, 'b')
    ax1.semilogx(w/(2*np.pi), mag_cl, 'c')    
    ax1.set_title('Open Loop Blue, Close Loop Cyan')
    ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
    ax1.set_xlabel('Frequency [Hz]')
    ax1.set_ylim(ymin=-40, ymax=40)

    ax2.semilogx(w/(2*np.pi), phase_ol, 'b')
    ax2.semilogx(w/(2*np.pi), phase_cl, 'c')    
    ax2.set_ylabel('Phase', color='b')
    ax2.set_xlabel('Frequency [Hz]')

    plt.tight_layout()
    plt.show()


if Polos_Ceros_CloseLoop_Digital == True:
    plot_argand(close_loop_dig)

##########################################################
# Verifico Respuesta Escalon Planta Digital Realimentada #
##########################################################
tiempo_de_simulacion = 6.0
t = np.linspace(0, tiempo_de_simulacion, num=(tiempo_de_simulacion*Fsampling))
tout, yout_zoh = dstep([close_loop_dig.num, close_loop_dig.den, td], t=t)
yout1 = np.transpose(yout_zoh)
yout0 = yout1[0]
yout_zoh = yout0[:tout.size]

tout, yout_planta_dig_zoh = dstep([planta_dig_zoh.num, planta_dig_zoh.den, td], t=t)
yout1 = np.transpose(yout_planta_dig_zoh)
yout0 = yout1[0]
yout_planta_dig_zoh = yout0[:tout.size]


if Escalon_CloseLoop_Digital == True:
    fig, ax = plt.subplots()
    ax.set_title('Step Planta Digital Realimentada')
    ax.set_ylabel('Output')
    ax.set_xlabel('Time [s]')
    ax.grid()
    ax.plot(tout, yout_zoh, color='orange')

    plt.tight_layout()
    plt.show()

if Escalon_CloseLoop_Original_Digital == True:
    fig, ax = plt.subplots()
    ax.set_title('Step Planta Digital Realimentada Orange y Original ZOH Yellow')
    ax.set_ylabel('Output')
    ax.set_xlabel('Tiempo [s]')
    ax.grid()
    ax.plot(tout, yout_planta_dig_zoh, 'y')
    ax.plot(tout, yout_zoh, color='orange')

    plt.tight_layout()
    plt.show()


