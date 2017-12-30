# -*- coding: utf-8 -*-
import numpy as np
from scipy import *
from pylab import *

import scipy.signal as signal


#sample time
Ts = 0.0005

#vector lenght
lenght = 20

#time vector
t = np.arange (0, lenght * Ts, Ts)

figure(1)
clf()# clear the figure - "#" is the comment symbol in Python


#current vector
imax = 1
it = imax * sin(2*pi*50*t)
itd = imax * sin(2*pi*50*t+pi/40)
#itd = imax * sin(2*pi*50*t)

ylabel('Current')

x = arange(0,20)
#stem(x, it, linefmt='b-')
stem(x, itd, linefmt='r-')
plot(x,it,'y')
show()