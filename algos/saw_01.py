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

freq = np.arange (10e6, 10e9, 10e6)  #de 40 a 1000MHz cada 10MHz
#freq = np.arange (400e6, 500e6, 1e6)    #de 400 a 500MHz cada 1MHz
#Zp = 
#Zt = 1. / (1./Zm + 1./Zp)


#print Zt.simplify()

H = 1/s * (s**2 * Lm * Cm + s * Rm * Cm + 1) / (s**2 * Lm * Cm * Cp + s * Rm * Cm * Cp + Cm + Cp)

print (H.simplify())

#convierto a lti con Lm = 86uHy y Cp 2.3pF
#num = [1.29e-16, 3.9e-11, 1]
#den = [2.967e-28, 8.97e-23, 3.8e-12, 0]

#convierto a lti con Lm = 86uHy y Cp 1.9pF
num = [1.29e-16, 3.9e-11, 1]
den = [2.451e-28, 7.41e-23, 3.4e-12, 0]

#convierto a lti con Lm = 86nHy
#num = [1.29e-19, 3.9e-11, 1]
#den = [2.967e-31, 8.97e-23, 3.8e-12, 0]

saw = lti(num, den)

w, mag, phase = bode(saw, freq)

plt.figure(1)
plt.semilogx (w, mag, color="blue", linewidth="1")
plt.show()
