# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt

muestras = 150
Vmax = 465

s_cuad = np.ones(muestras)
s_cuad = s_cuad * Vmax

s_cuad_enteros = s_cuad.astype(int)
s_cuad_enteros[int(muestras/2):muestras] = 0

print (s_cuad_enteros)



### imprimo para codico C ###
cant_por_linea = 10
linea = 1

print ("{",end='')
for i in range(np.size(s_cuad_enteros)):
    if i < (linea * cant_por_linea):
        if i == (np.size(s_cuad_enteros) - 1):
            print (str(s_cuad_enteros[i]),end='')
        else:                
            print (str(s_cuad_enteros[i]) + ",",end='')
    else:
        print ("\n",end='')
        linea += 1
        
print ("};")
