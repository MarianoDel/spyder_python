# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt

muestras = 150
Vmax = 465

s_sen = np.zeros(muestras)

for i in range(np.size(s_sen)):
    s_sen[i] = np.sin(2*np.pi*i/muestras) * Vmax

s_sen_enteros = s_sen.astype(int)
s_sen_enteros[int(muestras/2):muestras] = 0

print (s_sen_enteros)



### imprimo para codico C ###
cant_por_linea = 10
linea = 1

print ("{",end='')
for i in range(np.size(s_sen_enteros)):
    if i < (linea * cant_por_linea):
        if i == (np.size(s_sen_enteros) - 1):
            print (str(s_sen_enteros[i]),end='')
        else:                
            print (str(s_sen_enteros[i]) + ",",end='')
    else:
        print ("\n",end='')
        linea += 1
        
print ("};")
