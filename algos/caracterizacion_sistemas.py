# -*- coding: utf-8 -*-
#usar python3
import numpy as np
from sympy import *
import matplotlib.pyplot as plt
from scipy.signal import lti, bode, lsim, step2, TransferFunction
from tc_udemm import sympy_to_lti, lti_to_sympy, plot_s_plane

"""
        Caracterizo varios sistemas en Frecuencia
        luego realimento unity-feedback para ver la respuesta que consigo

"""

##########################################################################
# Cuales son los Graficos que quiero mostrar por cuestiones de velocidad #
##########################################################################
Frequency_Response = False
Step_Response = False
Poles_and_Zeros_Diagram = False
CloseLoop_Frequency_Response = False
CloseLoop_Poles_and_Zeros_Diagram = False
Frequency_Response_OpenLoop_CloseLoop = True
CloseLoop_Step_Response = True



# Complex variable
s = Symbol('s')

# integrador ganancia y polo
num = (s + 62.8)
den = s*(s + 628)
gain = 800*1
plant_sympy = gain * num / den

fmin = 0.1
fmax = 10000
tiempo_de_simulacion = 3.0

plant_tf = sympy_to_lti(plant_sympy)
print (plant_tf)

######################
# Frequency Response #
######################
if Frequency_Response == True:
    wfreq = np.arange(fmin*2*np.pi, fmax*2*np.pi, fmin)

    w, mag_ol, phase_ol = bode(plant_tf, wfreq)

    fig, (ax1, ax2) = plt.subplots(2,1)
    ax1.semilogx (w/6.28, mag_ol, 'b-', linewidth="1")
    ax1.set_title('Magnitude')

    ax2.semilogx (w/6.28, phase_ol, 'b-', linewidth="1")
    ax2.set_title('Phase')

    plt.tight_layout()
    plt.show()

    
#################
# Step Response #
#################
if Step_Response == True:
    t = np.linspace(0, tiempo_de_simulacion, num=2000)
    u = np.ones_like(t)
    t, y, x = lsim(plant_tf, u, t)

    fig, ax = plt.subplots()
    ax.set_title('Respuesta de la Planta vista desde el sensor')
    ax.set_ylabel('Vsensor')
    ax.set_xlabel('Tiempo [s]')
    ax.grid()
    ax.plot(t, y, 'b-')
    ax.plot(t, u, color='orange')

    plt.tight_layout()
    plt.show()


###########################
# Poles and Zeros Diagram #
###########################
if Poles_and_Zeros_Diagram == True:
    plot_s_plane(plant_tf)


#############################################
# Cierro el lazo Unity Feedback - CloseLoop #
#############################################
cl = plant_sympy/(1+plant_sympy)
close_loop_tf = sympy_to_lti(cl)


################################
# Frequency Response CloseLoop #
################################
if CloseLoop_Frequency_Response == True:
    wfreq = np.arange(fmin*2*np.pi, fmax*2*np.pi, fmin)

    w, mag_p, phase_p = bode(close_loop_tf, wfreq)

    fig, (ax1, ax2) = plt.subplots(2,1)
    ax1.semilogx (w/6.28, mag_p, 'y-', linewidth="1")
    ax1.set_title('Magnitude')

    ax2.semilogx (w/6.28, phase_p, 'y-', linewidth="1")
    ax2.set_title('Phase')

    plt.tight_layout()
    plt.show()




###########################
# Poles and Zeros Diagram #
###########################
if CloseLoop_Poles_and_Zeros_Diagram == True:
    plot_s_plane(close_loop_tf)


#########################################
# Frequency Response OpenLoop CloseLoop #
#########################################
if Frequency_Response_OpenLoop_CloseLoop == True:
    wfreq = np.arange(fmin*2*np.pi, fmax*2*np.pi, fmin)

    w, mag_ol, phase_ol = bode(plant_tf, wfreq)
    w, mag_cl, phase_cl = bode(close_loop_tf, wfreq)

    fig, (ax1, ax2) = plt.subplots(2,1)
    ax1.semilogx (w/6.28, mag_ol, 'b-', linewidth="1")
    ax1.semilogx (w/6.28, mag_cl, 'y-', linewidth="1")
    ax1.set_title('Magnitude')
    ax1.grid(True)

    ax2.semilogx (w/6.28, phase_ol, 'b-', linewidth="1")
    ax2.semilogx (w/6.28, phase_cl, 'y-', linewidth="1")
    ax2.set_title('Phase')
    ax2.grid(True)    

    plt.tight_layout()
    plt.show()


###########################
# Step Response CloseLoop #
###########################
if CloseLoop_Step_Response == True:
    t = np.linspace(0, tiempo_de_simulacion, num=2000)
    u = np.ones_like(t)
    t, y, x = lsim(close_loop_tf, u, t)

    fig, ax = plt.subplots()
    ax.set_title('Step CloseLoop')
    # ax.set_ylabel('Vsensor')
    ax.set_xlabel('Tiempo [s]')
    ax.grid()
    ax.plot(t, y, 'y-')
    ax.plot(t, u, color='orange')

    plt.tight_layout()
    plt.show()

