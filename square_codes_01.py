# -*- coding: utf-8 -*-
"""
Created on Sun Nov 20 16:43:34 2016

@author: med
"""

import random
import matplotlib.pyplot as plt
import numpy as np

N = 10
x_vals = range(N)
bit_vals = [-1, 1]
my_bits = []

tstep = 0.001   #en ms, 1us
tlambda = 330 * tstep #330us
len_lambda = 330
sigma_lambda = 30
len_pilot = 5000

#vector de tiempos
t = np.arange(0, 30, tstep)
signal = np.zeros_like(t)
signaln = np.zeros_like(t)

code_to_tx = np.array([1,1,1,0,0,0,1,1,1,0,0,0])


#ARMO LA SEÑAL
#pilot + start bit
end_of_start_bit = len_pilot + len_lambda
signal[len_pilot:end_of_start_bit] = 1.0
#wait bit
end_of_wait_bit = end_of_start_bit + len_lambda
signal[end_of_start_bit:end_of_wait_bit] = 0
#codes
offset = end_of_wait_bit
for x in range(code_to_tx.size):
    local_offset = offset+x*3*len_lambda
    if (code_to_tx[x] == 1):
        signal[local_offset:local_offset+2*len_lambda] = 0.0
        signal[local_offset+2*len_lambda:local_offset+3*len_lambda] = 1.0
    else:
        signal[local_offset:local_offset+len_lambda] = 0.0
        signal[local_offset+len_lambda:local_offset+3*len_lambda] = 1.0

#ARMO LA SEÑAL CON RUIDO
#pilot + start bit
noisy_lambda = len_lambda + sigma_lambda * np.random.randn()
end_of_start_bit = len_pilot + int(noisy_lambda)
signaln[len_pilot:end_of_start_bit] = 1.0
#wait bit
noisy_lambda = len_lambda + sigma_lambda * np.random.randn()
end_of_wait_bit = end_of_start_bit + int(noisy_lambda)
signaln[end_of_start_bit:end_of_wait_bit] = 0
#codes
offset = end_of_wait_bit
local_offset = 0
for x in range(code_to_tx.size):
    noisy_lambda = len_lambda + sigma_lambda * np.random.randn()
    #local_offset = offset+x*3*noisy_lambda
    local_offset += offset
    offset = int(3*noisy_lambda)
	two_noisy_lambda = int (2 * noisy_lambda)
	three_noisy_lambda= int (3 * noisy_lambda)

    if (code_to_tx[x] == 1):
        signaln[local_offset:local_offset+two_noisy_lambda] = 0.0
        signaln[local_offset+two_noisy_lambda:local_offset+three_noisy_lambda] = 1.0
    else:
        signaln[local_offset:local_offset+noisy_lambda] = 0.0
        signaln[local_offset+noisy_lambda:local_offset+three_noisy_lambda] = 1.0



plt.plot(t,signal)
plt.plot(t,signaln, 'g')

#for x in xrange(N):
#    my_bits.append(bit_vals[random.randint(0, 1)])
#print my_bits

#fig = plt.figure()
#ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
#ax.set_ylim(ymin=-2, ymax=2)
#ax.step(x_vals, my_bits, color='g')
#ax.grid()
#ax.set_yticks((-1, 1))

#plt.show()
