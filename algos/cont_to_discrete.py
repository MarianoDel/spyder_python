# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf, tf2zpk, step2, lsim
from scipy.signal import cont2discrete, freqz, ZerosPolesGain, TransferFunction
from tc_udemm import sympy_to_lti, lti_to_sympy
from math import pi, sin


"""
    Desde la funcion en s
    busco ceros polos y ganancia, luego digitalizo
"""

#ecuacion transferencia en s
s = Symbol('s')
f_s = (s + 1) / ((s + 2) * (s + 3))

final_value = f_s.subs(s, 0).evalf()
print ('Final value: ' + str(final_value))
#convierto a funcion lti
planta = sympy_to_lti(f_s)
print ('Numerador Planta Sympy: ' + str(planta.num))
print ('Denominador Planta Sympy: ' + str(planta.den))

#busco zeros polos y ganancia
z, p, k = tf2zpk(planta.num, planta.den)
# print ('Ceros: ' + str(planta.zeros))
print ('Ceros: ' + str(z))
# print ('Polos: ' + str(planta.poles))
print ('Polos: ' + str(p))
print ('K: ' + str(k))

### Desde aca utilizo ceros y polos que entrego sympy
### Convierto a sistema Digital
### Convierto Forward Euler
Fsampling = 5
Tsampling = 1 / Fsampling
num_d1, den_d1, td = cont2discrete((planta.num, planta.den), Tsampling, method='euler')
num_d2, den_d2, td = cont2discrete((planta.num, planta.den), Tsampling, method='euler')
# num_d1, den_d1, td = cont2discrete((planta.num, planta.den), Tsampling, method='zoh')
# num_d1, den_d1, td = cont2discrete((planta.num, planta.den), Tsampling, method='backward_diff')
# num_d1, den_d1, td = cont2discrete((planta.num, planta.den), Tsampling, method='bilinear')

print ('Numerador Digital planta: ' + str(num_d1))
print ('Denominador Digital planta: ' + str(den_d1))

#busco zeros polos y ganancia
# despues ajusto zeros y ganancia, por ultimo normalizo e imprimo
zd, pd, kd = tf2zpk(num_d1, den_d1)

while (np.shape(zd) < np.shape(pd)):
    zd = np.append(zd, [-1])

#normalizo
planta_d = ZerosPolesGain(zd, pd, kd)
planta_d = planta_d.to_tf()
zd, pd, kd = tf2zpk(planta_d.num, planta_d.den)

#convierto a sympy para evaluar el valor final
planta_d_sympy = lti_to_sympy(planta_d)
z = Symbol('z')
planta_d_sympy = planta_d_sympy.subs(s, z)
print (planta_d_sympy)
final_value_d = planta_d_sympy.subs(z, 1).evalf()
print ('final value digital: ' + str(final_value_d))

#ahora ajusto la ganancia para que me coincidan los dos valores finales
kd = kd * final_value / final_value_d
print ('Ceros digital: ' + str(zd))
print ('Polos digital: ' + str(pd))
print ('K digital: ' + str(kd))

#normalizo por ultima vez planta_d, ya agregue los zeros y ajuste la ganancia con los valores finales
planta_d = ZerosPolesGain(zd, pd, kd)
planta_d = planta_d.to_tf()

planta_d2 = TransferFunction(num_d2, den_d2)

       

## Comparo respuesta en frecuencia
w, h = freqz(planta.num, planta.den, worN=2000)
wd, hd = freqz(planta_d.num, planta_d.den, worN=2000)
wd2, hd2 = freqz(planta_d2.num, planta_d2.den, worN=2000)
fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(2*pi)*Fsampling, 20 * np.log10(abs(h)), 'b')
ax1.semilogx(wd/(2*pi)*Fsampling, 20 * np.log10(abs(hd)), 'g')
ax1.semilogx(wd2/(2*pi)*Fsampling, 20 * np.log10(abs(hd2)), 'y')
ax1.set_ylabel('Amplitude [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')

angles = np.unwrap(np.angle(h))
angles_d = np.unwrap(np.angle(hd))
angles_d2 = np.unwrap(np.angle(hd2))
ax2.semilogx (w/(2*pi)*Fsampling, angles*180/pi, 'b-', linewidth="1")
ax2.semilogx (wd/(2*pi)*Fsampling, angles_d*180/pi, 'g-', linewidth="1")
ax2.semilogx (wd2/(2*pi)*Fsampling, angles_d2*180/pi, 'y-', linewidth="1")
ax2.set_title('Angle')

plt.tight_layout()
plt.show()

