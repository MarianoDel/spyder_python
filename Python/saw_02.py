# -*- coding: utf-8 -*-
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import lti, step, bode

from sympy import *

#sympy.init_printing()
s = Symbol('s')

##FILTRO SAW HDR433
Rm = 26
Lm = 86e-6      #86uHy
#Lm = 86e-9     #86nHy    
Cm = 1.5e-12
#Cp = 2.3e-12
Cp = 1.9e-12

freq = np.arange (10e6, 1000e6, 1e6)  #de 40 a 1000MHz cada 10MHz
#freq = np.arange (400e6, 500e6, 1e6)    #de 400 a 500MHz cada 1MHz
Zm = Rm + s * Lm + 1. /(s * Cm)
Zp = 1. / (s * Cp)
#Zt = 1. / (1./Zm + 1./Zp)
Zt = Zm * Zp / (Zm + Zp)

num_z, den_z = fraction(Zt)

print "Zt = "
print cancel(Zt)
print num_z
print den_z

H = 1/s * (s**2 * Lm * Cm + s * Rm * Cm + 1) / (s**2 * Lm * Cm * Cp + s * Rm * Cm * Cp + Cm + Cp)

print "H = "
print factor(H)


#convierto a lti con Lm = 86uHy y Cp 2.3pF
#num = [1.29e-16, 3.9e-11, 1]
#den = [2.967e-28, 8.97e-23, 3.8e-12, 0]

#convierto a lti con Lm = 86uHy y Cp 1.9pF
numH = [1.29e-16, 3.9e-11, 1]
denH = [2.451e-28, 7.41e-23, 3.4e-12, 0]

#convierto a lti con Lm = 86nHy
#num = [1.29e-19, 3.9e-11, 1]
#den = [2.967e-31, 8.97e-23, 3.8e-12, 0]

#convierto a lti con Lm = 86uHy y Cp 1.9pF
numZ = [45263157.8947368, 13684210526315.8, 3.50877192982456e+23]
denZ = [8.6e-5, 26.0, 1192982456140.35, 0]

sawZ = lti(numZ, denZ)
wZ, magZ, phaseZ = bode(sawZ, freq)

sawH = lti(numH, denH)
wH, magH, phaseH = bode(sawH, freq)

plt.figure(1)
plt.semilogx (wH, magH, color="blue", linewidth="1")
plt.semilogx (wZ, magZ, color="green", linewidth="1")
plt.show()
