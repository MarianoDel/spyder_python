from scipy import signal
import matplotlib.pyplot as plt
import numpy as np

"""
   dbode devuelve un w = pi / dt, 100 puntos en general
   si le paso como parametro un w a evaluar, se lo debo pasar como relacion con pi
   por ejemplo 1000 puntos entre 0 y 0.1 pi w_eval = np.arange(0, np.pi /10, np.pi / 1000)

"""

#Transfer function: H(z) = 1 / (z^2 + 2z + 3)
# ts = 0.05        #original
ts = 1
sys = signal.TransferFunction([1], [1, 2, 3], dt=ts)

#Equivalent: sys.bode()
f_eval = np.arange(0, np.pi/10, np.pi / 10000)     #desde 0 a pi 1000 puntos
print (f_eval)
# w, mag, phase = signal.dbode(sys, w=f_eval*2*np.pi)
w, mag, phase = signal.dbode(sys, w=f_eval)
# w, mag, phase = signal.dbode(sys)           #original

plt.figure()
plt.semilogx(w, mag)    # Bode magnitude plot
plt.figure()
plt.semilogx(w, phase)  # Bode phase plot
plt.show()

print ("w en dbode: ")
print (w)
