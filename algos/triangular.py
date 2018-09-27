# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt

muestras = 150
Vmax = 465

s_triang = np.zeros(muestras)

mitad_largo = np.size(s_triang) / 2
mitad_largo = int(mitad_largo)

for i in range(mitad_largo+1):
    s_triang[i] = i * Vmax / (mitad_largo)

s_triang_enteros = s_triang.astype(int)
s_triang_enteros[int(mitad_largo+1):muestras] = 0

print (s_triang_enteros)



### imprimo para codico C ###
cant_por_linea = 10
linea = 1

print ("{",end='')
for i in range(np.size(s_triang_enteros)):
    if i < ((linea * cant_por_linea) - 1):
        print (str(s_triang_enteros[i]) + ",",end='')
    else:
        if i == (np.size(s_triang_enteros) - 1):
            print (str(s_triang_enteros[i]),end='')
        else:                
            print (str(s_triang_enteros[i]) + ",\n",end='')
            linea += 1
        
print ("};")
