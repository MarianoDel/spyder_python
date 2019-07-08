# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, bode, lsim, TransferFunction
from tc_udemm import sympy_to_lti, lti_to_sympy

"""
        Analog PID for params determination
"""

#################
# PID analogico #
#################
kp = 0.1
ki = 1
kd = 0.000001

s = Symbol('s')
Pi_out = kp + ki/s
Pi_out_sim = Pi_out.simplify()
Pd_out = kp + s*kd
Pd_out_sim = Pd_out.simplify()
Pid_out = kp + ki/s + s*kd
Pid_out_sim = Pid_out.simplify()

print ('PI_out: ')
print (Pi_out_sim)
print ('PD_out: ')
print (Pd_out_sim)
print ('PID_out: ')
print (Pid_out_sim)

##############################################
# Grafico de Bode con Polos y Ceros de sympy #
##############################################
pi = sympy_to_lti(Pi_out_sim)
pd = sympy_to_lti(Pd_out_sim)
pid = sympy_to_lti(Pid_out_sim)
print ("PI Tf:")
print (pi)
print ("PD Tf:")
print (pd)
print ("PID Tf:")
print (pid)

freq = np.arange(1, 1000000, 1)

w, mag_pi, phase_pi = bode(pi, freq)
w, mag_pd, phase_pd = bode(pd, freq)
w, mag_pid, phase_pid = bode(pid, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag_pi, '-b')
ax1.semilogx (w/(2*np.pi), mag_pd, '--y')
ax1.semilogx (w/(2*np.pi), mag_pid, ':r')
ax1.set_title('Magnitude PI blue - PD yellow - PID red')

ax2.semilogx (w/(2*np.pi), phase_pi, '-b')
ax2.semilogx (w/(2*np.pi), phase_pd, '--y')
ax2.semilogx (w/(2*np.pi), phase_pid, ':r')
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

