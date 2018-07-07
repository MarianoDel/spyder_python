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

Iout = TY1 / Rsense
Iout_sim = Iout.simplify()

Isense = TY1 * Tfiltro
Isense_sim = Isense.simplify()


print ('Iout: ')
print (Iout_sim)
print ('Isense: ')
print (Isense_sim)
final_value = Isense_sim.subs(s, 0).evalf()
print ('Isense_sim Final value: ' + str(final_value))


# print ('Vout: ')
# print (Vout_sim)
# print ('Plant_out: ')
# print (Plant_out_sim)

planta_real = sympy_to_lti(Isense_sim)
print ('Numerador Planta Sympy: ' + str(planta_real.num))
print ('Denominador Planta Sympy: ' + str(planta_real.den))

z, p, k = tf2zpk(planta_real.num, planta_real.den)
print ('Ceros: ' + str(planta_real.zeros))
print ('Polos: ' + str(planta_real.poles))
print ('K: ' + str(k))

planta_out = sympy_to_lti(Iout_sim)
print ('Numerador Salida Sympy: ' + str(planta_out.num))
print ('Denominador Salida Sympy: ' + str(planta_out.den))

z1, p1, k1 = tf2zpk(planta_out.num, planta_out.den)
print ('Ceros: ' + str(planta_out.zeros))
print ('Polos: ' + str(planta_out.poles))
print ('K: ' + str(k1))

### Desde aca utilizo ceros y polos que entrego sympy

freq = np.arange(1, 10000, 0.01)
w, mag, phase = bode(planta_real, freq)
# wc, magc, phasec = bode(control, freq)
# wo, mago, phaseo = bode(openl, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*pi), mag, 'b-', linewidth="1")
ax1.set_title('Magnitude')

ax2.semilogx (w/(2*pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

### Pruebo step con d=0.9
t = np.linspace(0, 0.01, num=2000)
u = np.ones_like(t)
u = u * (0.9*Vin-Vd)
t, y, x = lsim(planta_real, T=t, U=u)
t, y1, x1 = lsim(planta_out, T=t, U=u)

fig.clear()
fig, ax = plt.subplots()
ax.set_title('Respuesta escalon')
ax.set_ylabel('Vsense')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, y, 'r-')
ax.plot(t, y1, 'b-')

plt.tight_layout()
plt.show()

### Desde aca sistema Digital
### Convierto Forward Euler
Fsampling = 4800
Tsampling = 1 / Fsampling
num_d1, den_d1, td = cont2discrete((planta_real.num, planta_real.den), Tsampling, method='euler')
# num_d1, den_d1, td = cont2discrete((planta_real.num, planta_real.den), Tsampling, method='zoh')
# num_d1, den_d1, td = cont2discrete((planta_real.num, planta_real.den), Tsampling, method='backward_diff')
# num_d1, den_d1, td = cont2discrete((planta_real.num, planta_real.den), Tsampling, method='bilinear')
print ('Numerador Digital planta out 1 ' + str(num_d1))
print ('Denominador Digital planta out 1 ' + str(den_d1))

planta_d1 = lti(num_d1, den_d1)
planta_d1_zpk = planta_d1.to_zpk()
print ('Ceros Digitales ' + str(planta_d1_zpk.zeros))
print ('Polos Digitales ' + str(planta_d1_zpk.poles))
print ('Gain Digital ' + str(planta_d1_zpk.gain))

# convierto a planta_d2 ajusto al sistema digital 2 zeros en infinito y corrijo la ganancia
zd2, pd2, kd2 = tf2zpk(num_d1, den_d1)

while (np.shape(zd2) < np.shape(pd2)):
    zd2 = np.append(zd2, [-1])

#normalizo
planta_d2 = ZerosPolesGain(zd2, pd2, kd2)
planta_d2 = planta_d2.to_tf()
zd2, pd2, kd2 = tf2zpk(planta_d2.num, planta_d2.den)

#convierto a sympy para evaluar el valor final y ajustarlo
planta_d2_sympy = lti_to_sympy(planta_d2)
z = Symbol('z')
planta_d2_sympy = planta_d2_sympy.subs(s, z)
final_value_d2 = planta_d2_sympy.subs(z, 1).evalf()

#ahora ajusto la ganancia para que me coincidan los dos valores finales
kd2 = kd2 * final_value / final_value_d2
print ('Ceros digital: ' + str(zd2))
print ('Polos digital: ' + str(pd2))
print ('K digital: ' + str(kd2))

#normalizo por ultima vez planta_d2, ya agregue los zeros y ajuste la ganancia con los valores finales
planta_d2 = ZerosPolesGain(zd2, pd2, kd2)
planta_d2 = planta_d2.to_tf()
print ('planta_d2 ' + str(planta_d2))
planta_d2_sympy = lti_to_sympy(planta_d2)
z = Symbol('z')
planta_d2_sympy = planta_d2_sympy.subs(s, z)
final_value_d2 = planta_d2_sympy.subs(z, 1).evalf()
print ('planta_d2 final value: ' + str(final_value_d2))

#reconvierto planta_d1 a lti
planta_d1 = planta_d1.to_tf()

# en frecuencia
# w, h = freqz(num_d, den_d,worN=np.logspace(0, 4, 1000))
w, h = freqz(planta_d1.num, planta_d1.den)
w2, h2 = freqz(planta_d2.num, planta_d2.den)
fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(2*pi)*Fsampling, 20 * np.log10(abs(h)), 'b')
ax1.semilogx(w2/(2*pi)*Fsampling, 20 * np.log10(abs(h2)), 'g')
ax1.set_ylabel('Amplitude [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')

angles = np.unwrap(np.angle(h))
angles2 = np.unwrap(np.angle(h2))
ax2.semilogx (w/(2*pi)*Fsampling, angles*180/pi, 'b-', linewidth="1")
ax2.semilogx (w2/(2*pi)*Fsampling, angles2*180/pi, 'g-', linewidth="1")
ax2.set_title('Angle')

plt.tight_layout()
plt.show()


### Ahora voy a probar la respuesta escalon de los sistemas digitales
planta_d1= dlti(planta_d1.num, planta_d1.den, dt=td)
planta_d2= dlti(planta_d2.num, planta_d2.den, dt=td)

tfinal = 0.1
num = tfinal * Fsampling
t = np.linspace(0, tfinal, num=num)
u = np.ones_like(t)
u = u * (0.9*Vin-Vd)
tout, yout = dlsim(planta_d1, u=u, t=t)
tout2, yout2 = dlsim(planta_d2, u=u, t=t)

# print (td)
# planta_d1= dlti(planta_d1.num, planta_d1.den, dt=td)
# planta_d2= dlti(planta_d2.num, planta_d2.den, dt=td)
# tfinal = 0.1
# num = tfinal * Fsampling
# t = np.linspace(0, tfinal, num=num)
# tout, yout = step([planta_d2.num, planta_d2.den], T=t)
# # tout, yout = step2([planta_d1.num, planta_d1.den], T=t)
# # tout, yout = dstep(planta_d1, t=t)
# # tout, yout = dstep(planta_d2, t=t)
yout1 = np.transpose(yout)
yout0 = yout1[0]
yout = yout0[:tout.size]


fig, ax = plt.subplots()
ax.set_title('Respuesta escalon de la planta d2')
ax.set_ylabel('Corriente')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.stem(tout, yout, 'b-')
# ax.stem(t, y1, 'g-')

plt.tight_layout()
plt.show()


# ### Controlador por polos ceros y constante
# ### para errores menores a 2% ganancia 34dB
# ### busco BW 1000Hz para escalones en 1ms

# poles = [0]	#polo en w = 1000
# # poles = [-4.440 + 4.440j, -4.440 - 4.440j, -1.083 + 0.0j]
# # zeros = [100 + 0.0j, 0.0 + 0.0j, 0.0 + 0.0]
# # zeros = [-100 + 0.0j]	#zero en w = 100
# zeros = [-100]
# k = 10
# controller = zpk2tf(zeros, poles, k)

# w, mag, phase = bode(controller, freq)

# fig.clear()
# fig, (ax1, ax2) = plt.subplots(2,1)
# ax1.semilogx (w/(2*pi), mag, 'b-', linewidth="1")
# ax1.set_title('Magnitude')

# ax2.semilogx (w/(2*pi), phase, 'r-', linewidth="1")
# ax2.set_title('Phase')

# plt.tight_layout()
# plt.show()

# from tc_udemm import multiplico_sistemas, sumo_sistemas, realimento
# open_loop = multiplico_sistemas(controller, planta)

# w, mag, phase = bode(open_loop, freq)

# fig.clear()
# fig, (ax1, ax2) = plt.subplots(2,1)
# ax1.semilogx (w/(2*pi), mag, 'b-', linewidth="1")
# ax1.set_title('Magnitude')

# ax2.semilogx (w/(2*pi), phase, 'r-', linewidth="1")
# ax2.set_title('Phase')

# plt.tight_layout()
# plt.show()


# closed_loop = realimento(open_loop, ([1],[1]))
# w, mag, phase = bode(closed_loop, freq)

# z, p, k = tf2zpk(closed_loop.num, closed_loop.den)
# print ('Ceros: ' + str(closed_loop.zeros))
# print ('Polos: ' + str(closed_loop.poles))
# print ('K: ' + str(k))

# fig.clear()
# fig, (ax1, ax2) = plt.subplots(2,1)
# ax1.semilogx (w/(2*pi), mag, 'b-', linewidth="1")
# ax1.set_title('Magnitude')

# ax2.semilogx (w/(2*pi), phase, 'r-', linewidth="1")
# ax2.set_title('Phase')

# plt.tight_layout()
# plt.show()

# #
# #
# ### Desde aca hago pruebas temporales
# t = np.linspace(0, 0.01, num=2000)
# t, y = step2(closed_loop, T=t)

# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Respuesta escalon de la función transferencia luego del PID realimentado')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)


# plt.tight_layout()
# plt.show()

# ### respuestas a funciones especificas CUADRADA 100Hz
# t = np.linspace(0, 0.02, num=2000)
# u = np.ones_like(t)
# u[500:1000] = 0
# u[1500:2000] = 0

# tout, y, x = lsim(closed_loop, u, t)
# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Salida sistema Cuadrada 100Hz')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
# ax.plot(t, u, 'g--')

# plt.tight_layout()
# plt.show()

# ### respuestas a funciones especificas CUADRADA 30Hz
# t = np.linspace(0, 0.066, num=2000)
# u = np.ones_like(t)
# u[500:1000] = 0
# u[1500:2000] = 0

# tout, y, x = lsim(closed_loop, u, t)
# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Salida sistema Cuadrada 30Hz')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
# ax.plot(t, u, 'g--')

# plt.tight_layout()
# plt.show()

# ### respuestas a funciones especificas CUADRADA 10Hz
# t = np.linspace(0, 0.2, num=2000)
# u = np.ones_like(t)
# u[500:1000] = 0
# u[1500:2000] = 0

# tout, y, x = lsim(closed_loop, u, t)
# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Salida sistema Cuadrada 10Hz')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
# ax.plot(t, u, 'g--')

# plt.tight_layout()
# plt.show()

# ### respuestas a funciones especificas SAWTOOTH 100Hz
# t = np.linspace(0, 0.02, num=2000)
# u = np.ones_like(t)
# u[500:1000] = 0
# u[1500:2000] = 0
# for i in list (range(500)):
# 	u[i] = i / 500

# for i in list (range(500)):
# 	u[i+1000] = i / 500


# tout, y, x = lsim(closed_loop, u, t)
# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Salida sistema Sawtooth 100Hz')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
# ax.plot(t, u, 'g--')

# plt.tight_layout()
# plt.show()

# ### respuestas a funciones especificas SAWTOOTH 30Hz
# t = np.linspace(0, 0.066, num=2000)
# u = np.ones_like(t)
# u[500:1000] = 0
# u[1500:2000] = 0
# for i in list (range(500)):
# 	u[i] = i / 500

# for i in list (range(500)):
# 	u[i+1000] = i / 500


# tout, y, x = lsim(closed_loop, u, t)
# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Salida sistema Sawtooth 30Hz')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
# ax.plot(t, u, 'g--')

# plt.tight_layout()
# plt.show()

# ### respuestas a funciones especificas SAWTOOTH 10Hz
# t = np.linspace(0, 0.2, num=2000)
# u = np.ones_like(t)
# u[500:1000] = 0
# u[1500:2000] = 0
# for i in list (range(500)):
# 	u[i] = i / 500

# for i in list (range(500)):
# 	u[i+1000] = i / 500


# tout, y, x = lsim(closed_loop, u, t)
# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Salida sistema Sawtooth 10Hz')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
# ax.plot(t, u, 'g--')

# plt.tight_layout()
# plt.show()

# ### respuestas a funciones especificas SENOIDAL 100Hz
# t = np.linspace(0, 0.02, num=2000)
# u = np.ones_like(t)
# u[500:1000] = 0
# u[1500:2000] = 0
# for i in list (range(500)):
# 	u[i] = sin(pi*i/500)

# for i in list (range(500)):
# 	u[i+1000] = sin(pi*i/500)


# tout, y, x = lsim(closed_loop, u, t)
# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Salida sistema Senoidal 100Hz')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
# ax.plot(t, u, 'g--')

# plt.tight_layout()
# plt.show()

# ### respuestas a funciones especificas SENOIDAL 30Hz
# t = np.linspace(0, 0.066, num=2000)
# u = np.ones_like(t)
# u[500:1000] = 0
# u[1500:2000] = 0
# for i in list (range(500)):
# 	u[i] = sin(pi*i/500)

# for i in list (range(500)):
# 	u[i+1000] = sin(pi*i/500)


# tout, y, x = lsim(closed_loop, u, t)
# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Salida sistema Senoidal 30Hz')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
# ax.plot(t, u, 'g--')

# plt.tight_layout()
# plt.show()

# ### respuestas a funciones especificas SENOIDAL 10Hz
# t = np.linspace(0, 0.2, num=2000)
# u = np.ones_like(t)
# u[500:1000] = 0
# u[1500:2000] = 0
# for i in list (range(500)):
# 	u[i] = sin(pi*i/500)

# for i in list (range(500)):
# 	u[i+1000] = sin(pi*i/500)


# tout, y, x = lsim(closed_loop, u, t)
# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Salida sistema Senoidal 10Hz')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
# ax.plot(t, u, 'g--')

# plt.tight_layout()
# plt.show()
