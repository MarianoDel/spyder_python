import matplotlib.pyplot as plt
import numpy as np


freq_i = 50				#frecuencia de linea
Ipeak = 0.4				#tension pico de la corriente
Vzero_current = 1.8		#tension del eje 0

Inoise_peak = 0.068		#tension pico a pico del ruido

length = 350
tstep = 1.0 / 15600		#frecuencia y tiempo de muestreo

#vector de muestras
t = np.arange(0, length * tstep, tstep)

wi = 2 * np.pi * freq_i
Ipp = Ipeak * np.sin(wi*t)

Inoise = np.ones_like(t)
for i in range(np.size(t)):
	Inoise[i] = Inoise_peak * np.random.randn()


plt.plot(t,Ipp)
plt.plot(t,Inoise)
# plt.plot(t,signaln, 'g')


#fig = plt.figure()
#ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
#ax.set_ylim(ymin=-2, ymax=2)
#ax.step(x_vals, my_bits, color='g')
#ax.grid()
#ax.set_yticks((-1, 1))

plt.show()
