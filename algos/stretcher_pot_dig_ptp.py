# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf, tf2zpk, step2, cont2discrete, dstep, freqz, freqs, dlti, TransferFunction, dbode
from tc_udemm import sympy_to_lti, lti_to_sympy

"""
	Analisis Etapa Potencia Stretcher Magneto  - Analogica y Digital -
         para los valores usa los siguientes scripts previos:
         stretcher_caracterizacion_etapa_pot.py  ;solo la etapa de potencia (planta) analogica y digital
         stretcher_pid01.py stretcher_pid02.py   ;resultados de la planta con PID distintos solo analog
"""

### ARRANCO DIRECTAMENTE CON LA POTENCIA Y EL PID DIGITAL ###

#stretcher_caracterizacion_etapa_pot.py
#otro mas ajustado, agrega zero en 1 y compensa ganancia
num_d2 = [0.091, 0.091]
den_d2 = [1, -0.8861]

#normalizo con dlti
planta_d2 = dlti(num_d2, den_d2)

#stretcher_pid02.py
### Controlador por polos ceros y constante
### para errores menores a 2% ganancia 34dB
### busco BW 1000Hz para escalones en 1ms

poles = [0]	#polo en w = 1000
# poles = [-4.440 + 4.440j, -4.440 - 4.440j, -1.083 + 0.0j]
# zeros = [100 + 0.0j, 0.0 + 0.0j, 0.0 + 0.0]
# zeros = [-100 + 0.0j]	#zero en w = 100
zeros = [-100]
k = 10
b, a = zpk2tf(zeros, poles, k)

f_eval = np.arange(0.1, 1000, 0.5)
w, mag, phase = bode((b, a), w=f_eval*2*np.pi)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
ax1.set_title('Magnitude')

ax2.semilogx (w/(2*np.pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()



controller = lti(b, a)   #normalizo

### Convierto Controlador por Forward Euler
Fsampling = 1500
Tsampling = 1 / Fsampling
cont_n, cont_d, td = cont2discrete((controller.num, controller.den), Tsampling, method='euler')

#normalizo con TransferFunction
print (cont_n)
print (cont_d)
controller_d = TransferFunction(cont_n, cont_d, dt=td)
print (controller_d)

#dbode devuelve w = pi / dt, 100 puntos
f_eval = np.arange(0, 0.5, 0.0001)    #de 0 a 1 en saltos de 0.01 de fsampling
   
# w, mag, phase = dbode(controller_d, w=f_eval*np.pi)
w, mag, phase = dbode(controller_d, n = 1000)
# w, mag, phase = dbode(controller_d)
# print (w)

fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(np.pi), mag, 'b')
ax1.set_title('PID Euler')
ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')
ax1.set_ylim([0, 65])

ax2.semilogx(w/(np.pi), phase, 'r')
ax2.set_ylabel('Phase', color='r')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()

#Multiplico para OpenLoop
c = lti_to_sympy(controller_d)
p = lti_to_sympy(planta_d2)

ol = c * p

open_loop = sympy_to_lti(ol)
open_loop = TransferFunction(open_loop.num, open_loop.den, dt=td)   #normalizo

w, mag, phase = dbode(open_loop, n = 1000)

fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(np.pi), mag, 'b')
ax1.set_title('Digital OpenLoop')
ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')

ax2.semilogx(w/(np.pi), phase, 'r')
ax2.set_ylabel('Phase', color='r')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()

print ("Resultado open loop: ")
print (open_loop)


r = ol / (1 + ol)
closed_loop = sympy_to_lti(r)
closed_loop = TransferFunction(closed_loop.num, closed_loop.den, dt=td)   #normailizo
w, mag, phase = dbode(closed_loop, n = 1000)


fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(np.pi), mag, 'b')
ax1.set_title('Digital Closed Loop')
ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')


ax2.semilogx(w/(np.pi), phase, 'r')
ax2.set_ylabel('Phase', color='r')
ax2.set_xlabel('Frequency [Hz]')

plt.tight_layout()
plt.show()

print ("Resultado sistema: ")
print (closed_loop)

### Desde aca hago pruebas temporales
t = np.linspace(0, 0.01, num=2000)
tout, yout = dstep([closed_loop.num, closed_loop.den, td], t=t)
yout1 = np.transpose(yout)
yout0 = yout1[0]
yout = yout0[:tout.size]


fig, ax = plt.subplots()
ax.set_title('Respuesta escalon de la función transferencia luego del PID realimentado')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.stem(tout, yout)

plt.tight_layout()
plt.show()
