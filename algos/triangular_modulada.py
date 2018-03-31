# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt

muestras = 200
Vmax = 255

s_triang = np.zeros(muestras)

mitad_largo = np.size(s_triang) / 2
mitad_largo = int(mitad_largo)

for i in range(mitad_largo):
    s_triang[i] = i * Vmax / (mitad_largo)

for i in range(mitad_largo):
    s_triang[mitad_largo + i] = Vmax - i * Vmax / (mitad_largo)
    
s_triang_enteros = s_triang.astype(int)

print (s_triang_enteros)

### imprimo para codico C ###
cant_por_linea = 10
linea = 1

print ("{",end='')
for i in range(np.size(s_triang_enteros)):
    if i < (linea * cant_por_linea):
        if i == (np.size(s_triang_enteros) - 1):
            print (str(s_triang_enteros[i]),end='')
        else:                
            print (str(s_triang_enteros[i]) + ",",end='')
    else:
        print ("\n",end='')
        linea += 1
        
print ("};")
