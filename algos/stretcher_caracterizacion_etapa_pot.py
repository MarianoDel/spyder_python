# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf, tf2zpk, step2, cont2discrete, dstep, freqz, freqs, dlti
from tc_udemm import sympy_to_lti, lti_to_sympy

"""
	Caracterizacion etapa potencia
	Analogica y Digital
"""

#Elementos de Hardware que pueden ser moviles
#Caracteristica de la bobina
L = 141e-3
R = 24

#Alimentacion del PWM
Vpwm = 35

#Elementos del Hardware mayormente fijos en la placa
Rsense = 0.33
Aopamp = 1 + 2200 / 1000	#hace que la salida sea aprox. 1V/1A

#resultados de la etapa de potencia
s = Symbol('s')
Iout = Vpwm / (s*L + R + Rsense)
Vout = Iout * Rsense
Plant_out = Vout * Aopamp

Iout_sim = Iout.simplify()
Vout_sim = Vout.simplify()
Plant_out_sim = Plant_out.simplify()

print ('Plant_out: ')
print (Plant_out_sim)

### Desde aca utilizo ceros y polos que entrego sympy
planta = sympy_to_lti(Plant_out_sim)
print ("planta con sympy:")
print (planta)

freq = np.arange(1, 10000, 0.01)

w, mag, phase = bode(planta, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/6.28, mag, 'b-', linewidth="1")
ax1.set_title('Magnitude')

ax2.semilogx (w/6.28, phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()


### Desde aca hago pruebas temporales
t = np.linspace(0, 0.1, num=2000)
t, y = step2(planta, T=t)

fig.clear()
fig, ax = plt.subplots()
ax.set_title('Respuesta escalon solo planta')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, y)

plt.tight_layout()
plt.show()


### Desde aca sistema Digital
### Convierto Forward Euler
Fsampling = 1500
Tsampling = 1 / Fsampling
num_d1, den_d1, td = cont2discrete((planta.num, planta.den), Tsampling, method='euler')

#normalizo con dlti
planta_d1 = dlti(num_d1, den_d1)
print ('Planta Digital sys 1')
print (str(planta_d1.num), '/(', str(planta_d1.den), ')')


#respuesta escalon
t = np.arange (0, 0.1, td)
tout, yout = dstep([planta_d1.num, planta_d1.den, td], t=t)
yout1 = np.transpose(yout)
yout0 = yout1[0]
yout = yout0[:tout.size]


plt.figure(1)
plt.clf()
plt.title('digital Step Response')


plt.stem(tout,yout)
plt.show()

###otro mas ajustado, agrega zero en 1 y compensa ganancia
num_d2 = [0.091, 0.091]
den_d2 = [1, -0.8861]

#normalizo con lti
planta_d2 = lti(num_d2, den_d2)

print ('Numerador Digital sys 2')
print (planta_d2.num)
print ('Denominador Digital sys 2')
print (planta_d2.den)

tout, yout = dstep([planta_d2.num, planta_d2.den, td], t=t)
yout1 = np.transpose(yout)
yout0 = yout1[0]
yout = yout0[:tout.size]


plt.figure(1)
plt.clf()
plt.title('digital Step Response')

# print (yout)
plt.stem(tout,yout)
plt.show()

# en frecuencia segundo funcion transferencia dgital
# w, h = freqz(num_d, den_d,worN=np.logspace(0, 4, 1000))
w, h = freqz(planta_d1.num, planta_d1.den)
fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(2*pi)*Fsampling, 20 * np.log10(abs(h)), 'b')
ax1.set_title('Planta Euler')
ax1.set_ylabel('Amplitude P D1 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')

angles = np.unwrap(np.angle(h))
ax2.semilogx (w/(2*pi)*Fsampling, angles*180/pi, 'r-', linewidth="1")
ax2.set_title('Angle')

plt.tight_layout()
plt.show(block=False)

# en frecuencia
# w, h = freqz(num_d, den_d,worN=np.logspace(0, 4, 1000))
w, h = freqz(planta_d2.num, planta_d2.den)
fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(2*pi)*Fsampling, 20 * np.log10(abs(h)), 'b')
ax1.set_title('Planta Euler + ajuste')
ax1.set_ylabel('Amplitude P D2 [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')

angles = np.unwrap(np.angle(h))
ax2.semilogx (w/(2*pi)*Fsampling, angles*180/pi, 'r-', linewidth="1")
ax2.set_title('Angle')

plt.tight_layout()
plt.show()
