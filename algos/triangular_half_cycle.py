# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt

muestras = 100
Vmax = 465

s_triang = np.zeros(muestras)

for i in range(np.size(s_triang)):
    s_triang[i] = (i+1) * Vmax / (muestras)

s_triang_enteros = s_triang.astype(int)

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
