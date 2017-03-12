# -*- coding: utf-8 -*-
"""
Created on Sun Nov 20 16:43:34 2016

@author: med
"""

import random
import matplotlib.pyplot as plt
import numpy as np

#Funcion auxiliar algoritmo detector
#devuelve un vector con los tiempos de transicion; intervalos entre transciones
def LookForTransitions (signal):
    last_edge = 0
    transition = np.zeros(60)
    accum_time = 0
    tindex = 0
    for x in range(signal.size):
        if (signal[x] != last_edge):
            transition[tindex] = t[x] - accum_time
            accum_time = t[x]
            last_edge = signal[x]
            tindex += 1
    
    trans = np.zeros (np.count_nonzero(transition))
    trans = transition[0:trans.size]
    return trans
    
#devuelve un vector con los tiempos de transicion pero en el intervalo original
def LookForTransitions2 (signal):
    last_edge = 0
    transition = np.zeros(60)
    tindex = 0
    for x in range(signal.size):
        if (signal[x] != last_edge):
            transition[tindex] = t[x]
            last_edge = signal[x]
            tindex += 1
    
    trans = np.zeros (np.count_nonzero(transition))
    trans = transition[0:trans.size]
    return trans
    
    
#Funcion que valida un vector de transiciones revisando el lambda
#toma en cuenta solo transiciones viables para computar
#devuelve cantidad de transciones computadas y lambda promedio
def UpdateTransitions (transition):
    N_lambda = 0
    total_lambda = 0
    no_check = 0
    no_founded = np.zeros(60)
    nf_index = 0

    for x in range (transition.size):        
        if (transition[x] > (3 * tlambda)):
            no_check += 1
            no_founded[nf_index] = x
            nf_index += 1
        elif (transition[x] > (1.5 * tlambda)):
            N_lambda += 2
            total_lambda += transition[x]
        elif (transition[x] > (0.5 * tlambda)):
            N_lambda += 1
            total_lambda += transition[x]
        else:
            no_check += 1
            no_founded[nf_index] = x
            nf_index += 1

    if (N_lambda != 0):
        prom = total_lambda / N_lambda
    else:
        prom = 0.0
    print prom
    print no_check

    ncheck = np.zeros (no_check)
    ncheck = no_founded[0:ncheck.size]
    print ncheck
    
    return N_lambda, prom


N = 10
x_vals = range(N)
bit_vals = [-1, 1]
my_bits = []

tstep = 0.001   #en ms, 1us

len_lambda = 330

tlambda = len_lambda * tstep #330us

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
#codes
offset = end_of_start_bit
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
end_of_start_bit = len_pilot + noisy_lambda
signaln[len_pilot:end_of_start_bit] = 1.0
#codes
offset = end_of_start_bit
local_offset = 0
for x in range(code_to_tx.size):
    noisy_lambda = len_lambda + sigma_lambda * np.random.randn()
    #local_offset = offset+x*3*noisy_lambda
    local_offset += offset
    offset = 3*noisy_lambda
    if (code_to_tx[x] == 1):        
        signaln[local_offset:local_offset+2*noisy_lambda] = 0.0
        signaln[local_offset+2*noisy_lambda:local_offset+3*noisy_lambda] = 1.0
    else:
        signaln[local_offset:local_offset+noisy_lambda] = 0.0
        signaln[local_offset+noisy_lambda:local_offset+3*noisy_lambda] = 1.0
        
    
plt.figure(1)
plt.clf()
plt.plot(t,signal)
plt.plot(t,signaln, 'g')

#ALGORITMO DETECTOR
#busco transciones y lambda
tr = LookForTransitions(signaln)
print tr

cant_tr, mu = UpdateTransitions(tr)
print "transiciones " + str (cant_tr)
print "lambda promedio " + str (mu)
    

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