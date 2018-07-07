# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf, tf2zpk, step2, lsim
from scipy.signal import cont2discrete, freqz, ZerosPolesGain, dstep, dlti, dlsim
from tc_udemm import sympy_to_lti, lti_to_sympy
from math import pi, sin


"""
    Primero comparo el lazo en s (tiempo continuo) con el modelo de LTSpice
    Digitalizo el modelo y aplico el control digital
"""

### funcion de ayuda al metodo de conversion forward euler (ojo con el mapeo es inestable!)
# convierto a planta_d2 ajusto al sistema digital 2 zeros en infinito y corrijo la ganancia
def convert_forward_euler (pa, Tsampling=0.01):
    # if isintance(pa, TransferFunction):
    if isinstance(pa, lti):

        #reviso primero el valor final
        s = Symbol('s')
        pa_sympy = lti_to_sympy(pa)
        final_value_analog = pa_sympy.subs(s, 0).evalf()
        # print (' Final value: ' + str(final_value_analog))

        #convierto backward euler
        num_d, den_d, td = cont2discrete((pa.num, pa.den), Tsampling, method='euler')
        
        zd, pd, kd = tf2zpk(num_d, den_d)
        #agrego zeros infinitos
        while (np.shape(zd) < np.shape(pd)):
            zd = np.append(zd, [-1])

        #normalizo los zeros
        planta_d = ZerosPolesGain(zd, pd, kd)
        planta_d = planta_d.to_tf()
        zd, pd, kd = tf2zpk(planta_d.num, planta_d.den)

        #convierto a sympy para evaluar el valor final y ajustarlo
        planta_d_sympy = lti_to_sympy(planta_d)
        z = Symbol('z')
        planta_d_sympy = planta_d_sympy.subs(s, z)
        final_value_d = planta_d_sympy.subs(z, 1).evalf()

        #ahora ajusto la ganancia para que me coincidan los dos valores finales
        kd = kd * final_value / final_value_d
        # print ('Ceros digital: ' + str(zd))
        # print ('Polos digital: ' + str(pd))
        # print ('K digital: ' + str(kd))

        #normalizo por ultima vez planta_d, ya agregue los zeros
        #y ajuste la ganancia con los valores finales
        planta_d = ZerosPolesGain(zd, pd, kd)
        planta_d = planta_d.to_tf()
        # print ('planta_d ' + str(planta_d))

        #muestro el valor final
        planta_d_sympy = lti_to_sympy(planta_d)
        z = Symbol('z')
        planta_d_sympy = planta_d_sympy.subs(s, z)
        final_value_d = planta_d_sympy.subs(z, 1).evalf()
        # print ('planta_d final value: ' + str(final_value_d))

        #reconvierto planta_d a dlti
        planta_d = dlti(planta_d.num, planta_d.den, dt=td)
        return planta_d

    else:
        raise ValueError('planta_analog is not instance of TransferFunction!')




#Elementos de Hardware segun modelo de LTSpice
L = 180e-6
Rsense = 0.33
Rf = 10e3
Cf = 47e-9

#leds segun modelo
Vd = 23.3
Rd = 2.93

#Alimentacion del PWM
Vin = 35

#etapa de potencia
s = Symbol('s')
TY1 = Rsense / (s*L + Rd + Rsense)

#etapa de filtro
Tfiltro = 1 / (s*Cf*Rf + 1)

# Iout = TY1 / Rsense
# Iout_sim = Iout.simplify()

Isense = TY1 * Tfiltro
Isense_sim = Isense.simplify()


# print ('Iout: ')
# print (Iout_sim)
print ('Isense: ')
print (Isense_sim)
final_value = Isense_sim.subs(s, 0).evalf()
print ('Isense_sim Final value: ' + str(final_value))

planta = sympy_to_lti(Isense_sim)
print ('Numerador Planta Sympy: ' + str(planta.num))
print ('Denominador Planta Sympy: ' + str(planta.den))

### Desde aca utilizo ceros y polos que entrego sympy
freq = np.arange(1, 10000, 0.01)
w, mag, phase = bode(planta, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*pi), mag, 'b-', linewidth="1")
ax1.set_title('Magnitude')

ax2.semilogx (w/(2*pi), phase, 'b-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

### Pruebo step
t = np.linspace(0, 0.01, num=2000)
u = np.ones_like(t)
t, y, x = lsim(planta, T=t, U=u)

fig.clear()
fig, ax = plt.subplots()
ax.set_title('Respuesta escalon')
ax.set_ylabel('Vsense')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, y, 'b-')

plt.tight_layout()
plt.show()

### Desde aca sistema Digital
### Convierto Forward Euler
Fsampling = 10000
Tsampling = 1 / Fsampling
num_d, den_d, td = cont2discrete((planta.num, planta.den), Tsampling, method='euler')
# num_d, den_d, td = cont2discrete((planta.num, planta.den), Tsampling, method='zoh')
# num_d, den_d, td = cont2discrete((planta.num, planta.den), Tsampling, method='backward_diff')
# num_d, den_d, td = cont2discrete((planta.num, planta.den), Tsampling, method='bilinear')

# p_digital = convert_forward_euler(planta, Tsampling)
# num_d = p_digital.num
# den_d = p_digital.den
# td = p_digital.dt

print ('Numerador Digital planta out 1 ' + str(num_d))
print ('Denominador Digital planta out 1 ' + str(den_d))
planta_d = dlti(num_d, den_d, dt=td)

#convierto a sympy para evaluar el valor final
planta_d_sympy = lti_to_sympy(planta_d)
z = Symbol('z')
planta_d_sympy = planta_d_sympy.subs(s, z)
final_value_d = planta_d_sympy.subs(z, 1).evalf()
print ('planta_d final value: ' + str(final_value_d))

# en frecuencia
w, h = freqz(planta_d.num, planta_d.den)
fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(2*pi)*Fsampling, 20 * np.log10(abs(h)), 'b')
ax1.set_ylabel('Amplitude [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')

angles = np.unwrap(np.angle(h))
ax2.semilogx (w/(2*pi)*Fsampling, angles*180/pi, 'b-', linewidth="1")
ax2.set_title('Angle')

plt.tight_layout()
plt.show()

### Ahora voy a probar la respuesta escalon del sistema digital
tfinal = 0.01
num = tfinal * Fsampling
t = np.linspace(0, tfinal, num=num)
tout, yout = dstep(planta_d, t=t)

yout1 = np.transpose(yout)
yout0 = yout1[0]
yout = yout0[:tout.size]

fig, ax = plt.subplots()
ax.set_title('Respuesta escalon de la planta d2')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.stem(tout, yout, 'b-')

plt.tight_layout()
plt.show()

