import sympy
from scipy import signal
from sympy import *

from pylab import *

from control import *
from control import matlab
import matplotlib.pyplot as plt

        
sympy.init_printing()

#POLO EN ORIGEN Y DOS CEROS (implementacion teorica del PID)

#desde el algoritmo hacia atras
#uk = uk-1 + k1 ek + k2 ek-1 + k3 ek-2
#Uz/Ez = (b0 + b1 z-1 + b2 z-2) / (1 - z-1)
#b0 = kp + kd + ki
#b1 = -kp - 2kd
#b2 = kd
#a0 = 1
#a1 = -1

fs = 44000
kp = 4.0
kd = 0.0
ki = 0.0156

b = [kp + kd + ki, -kp - 2*kd, kd]     #del spice
a = [1, -1]

print ("b vale")
print (b)
print ("a vale")
print (a)

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
