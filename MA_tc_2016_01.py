# -*- coding: utf-8 -*-
import numpy as np
from scipy import *
##from pylab import *
import matplotlib.pylab as plt
#import scipy.signal as signal


mu, sigma = 0, 0.15 # mean and standard deviation
input_rand = np.random.normal(mu, sigma, 1000);

input_vect = np.linspace(0,4*np.pi,1000);

input_sin = sin(input_vect);

input_adding = input_sin + input_rand;

plt.figure(1)
plt.plot(input_sin);
plt.plot(input_rand);
plt.show()

plt.figure(2)
plt.plot(input_adding);
#plt.show()

output_vect = np.zeros(1000);
#filtro MA de 8 taps
for index in range(size(input_adding)):
    if (index >= 7):
        output_vect[index] = input_adding[index] + input_adding[index - 1] + input_adding[index - 2] + input_adding[index - 3]
        output_vect[index] += input_adding[index - 4] + input_adding[index - 5] + input_adding[index - 6] + input_adding[index - 7]
        output_vect[index] /= 8.0

#plt.plot(output_vect);
plt.plot(output_vect+2);    #pongo offset para que se vea mejor
plt.show()
    