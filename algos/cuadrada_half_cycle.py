# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt

muestras = 100
Vmax = 558

s_cuad = np.ones(muestras)
s_cuad = s_cuad * Vmax
s_cuad_enteros = s_cuad.astype(int)

print (s_cuad_enteros)


### imprimo para codico C ###
cant_por_linea = 10
linea = 1

print ("{",end='')
for i in range(np.size(s_cuad_enteros)):
    if i < ((linea * cant_por_linea) - 1):
        print (str(s_cuad_enteros[i]) + ",",end='')
    else:
        if i == (np.size(s_cuad_enteros) - 1):
            print (str(s_cuad_enteros[i]),end='')
        else:                
            print (str(s_cuad_enteros[i]) + ",\n",end='')
            linea += 1
        
print ("};")
