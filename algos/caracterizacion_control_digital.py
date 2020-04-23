# -*- coding: utf-8 -*-
#usar python3
import numpy as np
from sympy import *
import matplotlib.pyplot as plt
from scipy.signal import lti, bode, lsim, dbode, zpk2tf, tf2zpk, step2, cont2discrete, dstep, freqz, freqs, dlti, TransferFunction
from tc_udemm import sympy_to_lti, lti_to_sympy, plot_argand


""" 
        Para caracterizar controles digitales custom
        Ubicacion Polos y Ceros y Respuesta en frecuencia

"""

########################################################
# Control Digital Custom, elijo Ceros Polos y Ganancia #
########################################################
Fsampling = 24000

cont_zeros = [0.916, -0.89+0.29j, -0.89-0.29j]
cont_poles = [1.0]
cont_const = 0.01
cont_zpk_b, cont_zpk_a = zpk2tf(cont_zeros, cont_poles, cont_const)

td = 1/Fsampling
controller_tf = TransferFunction(cont_zpk_b, cont_zpk_a, dt=td)

print ("Digital Controller:")
print (controller_tf)


#####################################
# Polos y Ceros del Control Digital #
#####################################
plot_argand(controller_tf)


########################
# Bode Control Digital #
########################
w, mag, phase = dbode(controller_tf, n = 10000)
fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx(w/(2*np.pi), mag, 'c')
ax1.set_title('Digital Controller TF')
ax1.set_ylabel('Amplitude [dB]', color='c')
ax1.set_xlabel('Frequency [Hz]')

ax2.semilogx(w/(2*np.pi), phase, 'c')
ax2.set_ylabel('Phase', color='c')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()


#--- end of file ---#
