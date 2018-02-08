# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt


"""
	Pruebo lazo PID solo con KP, los resultados del sistema los tomo punto a punto
	la ganancia del sistema la tomo de las mediciones.
	El lazo PID es backward-Euler
"""

#constantes del PID
KP = 3.2
KI = 0.0
KD = 0

K1 = (KP + KI + KD)
K2 = (KP + KD + KD)
K3 = (KD)

#globales
error_z1 = 0
error_z2 = 0

def PID_roof (setpoint, sample, last_d):
	global error_z1, error_z2

	error = setpoint - sample

	#K1
	val_k1 = K1 * error

	#K2
	val_k2 = K2 * error_z1

	#K3
	val_k3 = K3 * error_z2

	d = last_d + val_k1 - val_k2 + val_k3

	#Update variables PID
	error_z2 = error_z1
	error_z1 = error

	return d, error
	# return d

def PID_simple (setpoint, sample):

	error = setpoint - sample
	d = error * KP

	return d, error





#ganacia del sistema de las mediciones I_Sense = 69 cuando d = 224
I_Sense = 69
d = 224

gan_sistema = I_Sense / d

#vectores
sp = np.ones (1000)
sp = sp * 80


dpid = np.zeros_like (sp)
dpwm = np.zeros_like (sp)

e = np.zeros_like (sp)

iout = np.zeros_like (sp)

t = np.arange (0, np.size(sp), 1)

#verifico vectores, reducir size en sp para que no sea muy largo
# print ("sp", sp)
# print ("dpid", dpid)
# print ("dpwm", dpwm)
# print ("e", e)
# print ("iout", iout)
# print ("t", t)

#algoritmo
#for i, x in enumerate(sp):
for i in range(np.size(sp)):

	if i >= 1:		#corrijo primer punto
		dpid[i] , e [i] = PID_roof (sp[i], iout[i - 1], dpid[i - 1])
		# dpid[i] = PID_roof (sp[i], iout[i], dpid[i - 1])
		# dpid[i], e[i] = PID_simple (sp[i], iout[i - 1])

	if dpid [i] > 500:
		dpwm [i] = 500
	else:
		dpwm [i] = dpid [i]

	if dpid [i] < 0:
		dpwm[i] = 0

	# iout [i] = dpwm[i] * gan_sistema
	iout [i] = dpid[i] * gan_sistema


fig1, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5,1)     #API Matplotlib

ax1.plot(t,sp)
ax1.set_ylabel('SetPoint')

ax2.plot(t,iout)
ax2.set_ylabel('I_Out')

ax3.plot(t,e)
ax3.set_ylabel('Error')

ax4.plot(t,dpid)
ax4.set_ylabel('d_PID')

ax5.plot(t,dpwm)
ax5.set_ylabel('d_PWM')

plt.show()
