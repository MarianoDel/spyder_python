# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf, tf2zpk, step2, cont2discrete, dstep


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

print ('Iout: ')
print (Iout_sim)
print ('Vout: ')
print (Vout_sim)
print ('Plant_out: ')
print (Plant_out_sim)

### Desde aca utilizo ceros y polos que entrego sympy
num_planta = [0, 36.96]				#esto es b0 s1 y b1 s0
den_planta = [0.141, 24.33]			#esto es a0 s1 y a1 s0

planta = lti(num_planta, den_planta)

freq = np.arange(1, 10000, 0.01)

w, mag, phase = bode(planta, freq)
# wc, magc, phasec = bode(control, freq)
# wo, mago, phaseo = bode(openl, freq)

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
num_d, den_d, td = cont2discrete((num_planta, den_planta), 0.00066, method='euler')
print ('Numerador Digital')
print (num_d)
print ('Denominador Digital')
print (den_d)


#respuesta escalon
t = np.arange (0, 0.1, td)
tout, yout = dstep([num_d, den_d, td], t=t)
yout1 = np.transpose(yout)
yout0 = yout1[0]
yout = yout0[:tout.size]


plt.figure(1)
plt.clf()
plt.title('digital Step Response')

# print (yout)
plt.stem(tout,yout)
plt.show()

###otro mas ajustado
num_d = [0.091, 0.091]
den_d = [1, -0.8861]
tout, yout = dstep([num_d, den_d, td], t=t)
yout1 = np.transpose(yout)
yout0 = yout1[0]
yout = yout0[:tout.size]


plt.figure(1)
plt.clf()
plt.title('digital Step Response')

# print (yout)
plt.stem(tout,yout)
plt.show()
