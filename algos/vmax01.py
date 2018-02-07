import numpy as np
from scipy import *
from pylab import *

import matplotlib.pyplot as plt
import scipy.signal as signal


#vector length
length = 512
cycles = 2.
#time vector
t = np.arange (0, length)

figure(1)
clf()# clear the figure - "#" is the comment symbol in Python

#voltage vector
cyclesize = length / cycles
quartercycle = cyclesize / 4

vsense = np.zeros(length)

vsense[0 : quartercycle] = t[0:quartercycle]
vsense[quartercycle : 2*quartercycle] = vsense[quartercycle-1] - t[0:quartercycle]
vsense[2*quartercycle : 3*quartercycle] = vsense[2*quartercycle-1]-t[0:quartercycle]
vsense[3*quartercycle : 4*quartercycle] = vsense[3*quartercycle-1] + t[0:quartercycle]
    
vsense[4*quartercycle : length] = vsense[0 : 4*quartercycle]

plot(t,vsense)
ylabel('Voltage Sensed')

#busco el maximo
vmax = 0
for i, x in enumerate(vsense):
    if vmax <= vsense[i]:
        vmax = vsense[i]

#busco el minimo
vmin = 0
for i, x in enumerate(vsense):
    if vmin >= vsense[i]:
        vmin = vsense[i]

print (vmax)
print (vmin)

#subplot(212)
#vmax = 1
#theta_v = 5.5*pi/4
#vt = vmax * sin(2*pi*50*t+theta_v)
#plot(t,vt)
#ylabel('Voltage')

show()
draw()  #actualiza el grafico
