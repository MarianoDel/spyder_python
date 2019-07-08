# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, bode, lsim, TransferFunction
from tc_udemm import sympy_to_lti, lti_to_sympy

"""
        Analog PID for rapid testing
        with another pole for limit the Kd action
"""

#################
# PID analogico #
#################
kp = 0.01
ki = 1
kd = 0.000001
kpole = 0.0000001

kp_comp = kp*kpole

s = Symbol('s')
Pid_out = (kp + ki/s + s*kd)/(s + kpole)
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

