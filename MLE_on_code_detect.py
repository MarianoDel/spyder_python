# -*- coding: utf-8 -*-
"""
Created on Sun Nov 20 15:24:11 2016

@author: med

"""

##BITS
#los bits comienzan siempre con 0 y se conforman con 3 clks
#0 0->1clk, 1->2clk
#1 0->2clk, 1->1clk

##FRAME
#la trama se forma de 12bits de pilot code, todo en bajo
#un clk de sync en 1
#12 bits de codigo

##FREQUENCY
#la frecuencia tipica de clk es de 3KHz, puede ir de 1 a 7KHz

import matplotlib.pyplot as plt
import numpy as np

pilot_bits = 12
code_bits = 24
bits = pilot_bits + code_bits + 1
clks = bits * 3.0

transitions = clks * 2.0

clk_vect = np.zeros(transitions)
code_vect = np.zeros(transitions)

#numpy slicing
clk_vect [0::2] = 1.0

plt.figure(1)
#plt.plot(clk_vect.size(), clk_vect)
#plt.plot(clk_vect)
plt.step(clk_vect, 'b')

code_to_tx = np.array([1,1,1,0,0,0,1,1,1,0,0,0])

code_vect [pilot_bits*3*2] = 1.0

for x in range(code_to_tx.size):
    if (code_to_tx[x] == 1):
        code_vect[pilot_bits*3*2+1+x*3*2+0] = 0
        code_vect[pilot_bits*3*2+1+x*3*2+1] = 0
        code_vect[pilot_bits*3*2+1+x*3*2+2] = 1
    else:
        code_vect[pilot_bits*3*2+1+x*3*2+0] = 0
        code_vect[pilot_bits*3*2+1+x*3*2+1] = 1
        code_vect[pilot_bits*3*2+1+x*3*2+2] = 1


plt.step(code_vect+1.05, 'g')
    


        











