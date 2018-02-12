# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import pi
from scipy.signal import lti, bode, cont2discrete, freqz


"""
	Caracterizacion circuito RC
	Analogica y Digital
"""

#Caracteristica del circuito
R = 159.23
C = 1e-6

#Desde aca utilizo ceros y polos que entrego sympy
num = [6280.22]				#esto es b0 s1 y b1 s0 (orden descendente)
den = [1, 6280.22]				#esto es a0 s1 y a1 s0

planta = lti(num, den)

print ('Ceros')
print (planta.zeros)

print ('Polos')
print (planta.poles)

freq = np.arange(100, 1e5, 1)

w, mag, phase = bode(planta, freq)
# wc, magc, phasec = bode(control, freq)
# wo, mago, phaseo = bode(openl, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/6.28, mag, 'b-', linewidth="1")
ax1.set_title('Magnitude')

ax2.semilogx (w/6.28, phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show(block=False)

### Desde aca sistema Digital
### Convierto Forward Euler
Fsampling = 20000
Tsampling = 1 / Fsampling
num_d1, den_d1, td = cont2discrete((planta.num, planta.den), Tsampling, method='euler')
# num_d1, den_d1, td = cont2discrete((planta.num, planta.den), Tsampling, method='zoh')
# num_d1, den_d1, td = cont2discrete((planta.num, planta.den), Tsampling, method='backward_diff')
# num_d1, den_d1, td = cont2discrete((planta.num, planta.den), Tsampling, method='bilinear')
print ('Numerador Digital sys 1')
print (num_d1)
print ('Denominador Digital sys 1')
print (den_d1)

planta_d = lti(num_d1, den_d1)

print ('Ceros Digitales')
print (planta_d.zeros)

print ('Polos Digitales')
print (planta_d.poles)

# en frecuencia
# w, h = freqz(num_d, den_d,worN=np.logspace(0, 4, 1000))
w, h = freqz(planta_d.num, planta_d.den)
fig, (ax1, ax2) = plt.subplots(2,1)

ax1.semilogx(w/(2*pi)*Fsampling, 20 * np.log10(abs(h)), 'b')
ax1.set_ylabel('Amplitude [dB]', color='b')
ax1.set_xlabel('Frequency [Hz]')

angles = np.unwrap(np.angle(h))
ax2.semilogx (w/(2*pi)*Fsampling, angles*180/pi, 'r-', linewidth="1")
ax2.set_title('Angle')

plt.tight_layout()
plt.show()









#
#
# ### Desde aca hago pruebas temporales
# t = np.linspace(0, 0.1, num=2000)
# t, y = step2(planta, T=t)
#
# fig.clear()
# fig, ax = plt.subplots()
# ax.set_title('Respuesta escalon solo planta')
# ax.set_ylabel('Corriente')
# ax.set_xlabel('Tiempo [s]')
# ax.grid()
# ax.plot(t, y)
#
#
# plt.tight_layout()
# plt.show()
#
#
#
#
# #respuesta escalon
# t = np.arange (0, 0.1, td)
# tout, yout = dstep([num_d1, den_d1, td], t=t)
# yout1 = np.transpose(yout)
# yout0 = yout1[0]
# yout = yout0[:tout.size]
#
#
# plt.figure(1)
# plt.clf()
# plt.title('digital Step Response')
#
# # print (yout)
# plt.stem(tout,yout)
# plt.show()
#
# ###otro mas ajustado
# num_d2 = [0.091, 0.091]
# den_d2 = [1, -0.8861]
#
# print ('Numerador Digital sys 2')
# print (num_d2)
# print ('Denominador Digital sys 2')
# print (den_d2)
#
# tout, yout = dstep([num_d2, den_d2, td], t=t)
# yout1 = np.transpose(yout)
# yout0 = yout1[0]
# yout = yout0[:tout.size]
#
#
# plt.figure(1)
# plt.clf()
# plt.title('digital Step Response')
#
# # print (yout)
# plt.stem(tout,yout)
# plt.show()
#
# # en frecuencia
# # w, h = freqz(num_d, den_d,worN=np.logspace(0, 4, 1000))
# w, h = freqz(num_d2, den_d2)
# fig, (ax1, ax2) = plt.subplots(2,1)
#
# ax1.semilogx(w/6.28*1500, 20 * np.log10(abs(h)), 'b')
# ax1.set_ylabel('Amplitude [dB]', color='b')
# ax1.set_xlabel('Frequency [Hz]')
#
# angles = np.unwrap(np.angle(h))
# ax2.semilogx (w/6.28*1500, angles*180/3.1415, 'r-', linewidth="1")
# ax2.set_title('Angle')
#
# plt.tight_layout()
# plt.show()
#
# # en frecuencia segundo funcion transferencia dgital
# # w, h = freqz(num_d, den_d,worN=np.logspace(0, 4, 1000))
# num_d1 = [0.174, 0]
# den_d1 = [-0.885, 1]
# w, h = freqz(num_d1, den_d1)
# fig, (ax1, ax2) = plt.subplots(2,1)
#
# ax1.semilogx(w/6.28*1500, 20 * np.log10(abs(h)), 'b')
# ax1.set_ylabel('Amplitude [dB]', color='b')
# ax1.set_xlabel('Frequency [Hz]')
#
# angles = np.unwrap(np.angle(h))
# ax2.semilogx (w/6.28*1500, angles*180/3.1415, 'r-', linewidth="1")
# ax2.set_title('Angle')
#
# plt.tight_layout()
# plt.show()
