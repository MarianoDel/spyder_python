# -*- coding: utf-8 -*-
#usar python3
import numpy as np
from sympy import *
import matplotlib.pyplot as plt
from scipy.signal import lti, bode, lsim, dbode, zpk2tf, tf2zpk, step2, cont2discrete, dstep, freqz, freqs, dlti, TransferFunction
from tc_udemm import sympy_to_lti, lti_to_sympy, plot_argand

"""
        Caracterizacion de un Controlador PID digital
	Los parametros de entrada son:
        frecuencia de muestreo Fsampling
        KP KI KD digitales
        Muestra:
        Respuesta en frecuencia
        Diagrama de Polos y Ceros

"""

Fsampling = 24000

########################
# Ecuacion PID Digital #
########################
""" 
    Only for PID dig:
    w0 ~= ki_dig * Fsampling / kp_dig
    plateau gain ~= 20 log kp_dig
    w1 ~= kp_dig / (kd_dig * Fsampling) * 10    el 10 no se de donde sale???

"""
ki_dig = 0.00134
kp_dig = 0.02
kd_dig = 0.0

k1 = kp_dig + ki_dig + kd_dig
k2 = -kp_dig - 2*kd_dig
k3 = kd_dig

#este es el pid formado
b_pid = [k1, k2, k3]
a_pid = [1, -1]

td = 1/Fsampling
pid_dig_tf = TransferFunction(b_pid, a_pid, dt=td)


####################
# Bode PID Digital #
####################
w, mag, phase = dbode(pid_dig_tf, n = 10000)
fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx(w/(2*np.pi), mag, 'y')    
ax1.set_title('Digital PID')
ax1.set_ylabel('Amplitude [dB]', color='g')
ax1.set_xlabel('Frequency [Hz]')
# ax1.set_ylim(ymin=-40, ymax=40)

ax2.semilogx(w/(2*np.pi), phase, 'y')    
ax2.set_ylabel('Phase', color='g')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()


#############################
# Polos y Ceros PID Digital #
#############################
plot_argand(pid_dig_tf)

    
#--- end of file ---#
