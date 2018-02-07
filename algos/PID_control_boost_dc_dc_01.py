# -*- coding: utf-8 -*-
import sympy
from scipy import signal
from sympy import *

from pylab import *

from control import *
from control import matlab
import matplotlib.pyplot as plt

        
sympy.init_printing()

#CONSIDERACIONES PRACTICAS DE KI
#para lograr que ki ajuste algo (ej. 1 punto sobre el PWM)
#necesito que el error * ki = 1 entonces si ki = 0.01 el error minimo que voy a ajustar es de 100 puntos
#si 100 puntos de error es algo chico por ejemplo 0.01Amper no tengo problema, pero si es algo grande
#no voy a poder ajustar al SP deseado y debo subir ki hasta compensar
#simultaneamente subir ki puede traer problemas de inestabilidad, las puedo compensar si hago un undersampling
#entonces si ki > 1 / 16 es inestable puede hacer undersampling de 16 y poner ki en 1, con lo cual error minimo = 1

#CONSIDERACIONES PRACTICAS DE KP
#kp se suma al error actual y se resta en el error anterior
#k1 = kp + ki + kd; k2 = -kp - 2kd; k3 = kd
# si d = d_z1 + error * k1 + error_z1 * k2 + error_z3 * k3
# solo usando kp queda
# d = d_z1 + error * kp - error_z1 * kp
#si d_z1 no es grande y la frecuencia es alta, error y error_z1 pueden ser iguales
# con lo cual kp sale de la ecuacion y no puede regular o incluso no arranca el dispositivo ( a mayor freq peor es)
# se puede corregir con un ki pequeño que fuerza los primeros d_z1
# se puede corregir bajando la freq con undersampling


#POLO EN ORIGEN Y DOS CEROS (implementacion teorica del PID)

#desde el algoritmo hacia atras
#uk = uk-1 + k1 ek + k2 ek-1 + k3 ek-2
#Uz/Ez = (b0 + b1 z-1 + b2 z-2) / (1 - z-1)
#b0 = kp + kd + ki
#b1 = -kp - 2kd
#b2 = kd
#a0 = 1
#a1 = -1

#con undersampling de 10 es decir 4400Hz me da fc = 1463
fs = 4400.0
#kp = 1.0
#kd = 0.2
#ki = 0.0156
kp = 0.0
kd = 0.0
ki = 1.0

b = [kp + kd + ki, -kp - 2*kd, kd]     #del spice
a = [1, -1]

print ("b vale")
print (b)
print ("a vale")
print (a)

#de digital a analogico OJO ME DA UNA OCTAVA CORRIDO EL KI
ki_analog = ki * fs
kp_analog = kp + ki / 2
kd_analog = kd / fs

print ("ki_analog vale")
print (ki_analog)
print ("kp_analog vale")
print (kp_analog)
print ("kd_analog vale")
print (kd_analog)

#figure(1)
#mfreqz(b, a)
#show()


w, h = signal.freqz(b, a, 100000)
fig = plt.figure(1)
fig.clf()
plt.title('Digital filter frequency response')
ax1 = fig.add_subplot(111)

plt.semilogx(w*fs/pi, 20 * np.log10(abs(h)), 'b')
ax1.axis((1, 100000, -30, 30))
##plt.plot(w, 20 * np.log10(abs(h)), 'b')
##plt.plot(w / pi, abs(h), 'b')
plt.ylabel('Amplitude [dB]', color='b')
plt.xlabel('Frequency rad/sample')
plt.grid()
plt.show()
plt.draw()

##ax2 = ax1.twinx()
##angles = np.unwrap(np.angle(h))
##plt.semilogx(w*fs/(2*pi), angles, 'g')
##plt.plot(w, angles, 'g')
##plt.ylabel('Angle (radians)', color='g')
##plt.grid()
##plt.axis('tight')
##plt.show()

###Grafico con la nueva biblioteca control
figure(2)
clf()# clear the figure - "#" is the comment symbol in Python
#sys_new = tf(b, a, 1./fs)
sys_new = tf(b, a, True)
#sys_new = tf([1., 0.5] , a, 1./10)
#mag, phase, omega = bode_plot(sys_new, [0.1, 1000], True)
mag, phase, omega = bode_plot(sys_new, dB=True)
show() #muestra el grafico
draw()  #actualiza el grafico


###Grafico con Sympy
##figure(4)
##s = Symbol('s')
##G1 = 1 / (s + 1)
##freqResponse(G1)
