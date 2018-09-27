# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt

muestras = 150
Vmax = 465
# fase = -np.pi/2
fase = np.pi
# fase = 0

s_sen = np.zeros(muestras)
s_cuad = np.zeros(muestras)

for i in range(np.size(s_sen)):
    s_sen[i] = np.sin(2*np.pi*i/muestras + fase) * Vmax

s_sen_enteros = s_sen.astype(int)

for i in range (np.size(s_sen_enteros)):
    if s_sen_enteros[i] < 0:
        s_sen_enteros[i] = 0
        s_cuad[i] = 0
    else:
        s_cuad[i] = Vmax

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

# print ("{",end='')
# for i in range(np.size(s_cuad_enteros)):
#     if i < (linea * cant_por_linea):
#         if i == (np.size(s_cuad_enteros) - 1):
#             print (str(s_cuad_enteros[i]),end='')
#         else:                
#             print (str(s_cuad_enteros[i]) + ",",end='')
#     else:
#         print ("\n",end='')
#         linea += 1
