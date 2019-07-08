# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt

muestras = 50
Vmax = 1000
steps = 4    #incluye el 0, puede ser par o impar

s_sen = np.zeros(muestras)

total_segments = 2 * steps - 2
seg_length = int(muestras / total_segments)
seg_half_length = int(seg_length / 2)

loc_step = 0
for i in range(total_segments + 1):
    if (i == 0):
        s_sen[0:seg_half_length] = 0
    elif (i == total_segments):
        s_sen[i*seg_length + seg_half_length:total_segments * seg_length] = 0
    else:
        if (loc_step < steps):
            valor = Vmax * loc_step /(steps-1)
        else:
            valor = Vmax * (total_segments - loc_step) /(steps-1)

        s_sen[(i - 1)*seg_length + seg_half_length: i * seg_length + seg_half_length] = valor

    loc_step += 1


        
# s_sen[0:seg_half_length] = 0        
# s_sen[seg_half_length:seg_length + seg_half_length] = int(Vmax/(steps - 1))
# s_sen[seg_length + seg_half_length: 2 * seg_length + seg_half_length] = int(2*Vmax/(steps - 1))
# s_sen[2 * seg_length + seg_half_length: 3 * seg_length + seg_half_length] = int(Vmax/(steps - 1))
# s_sen[3 * seg_length + seg_half_length: total_segments * seg_length] = 0

s_sen_enteros = s_sen.astype(int)

print (s_sen_enteros)



### imprimo para codico C ###
cant_por_linea = 10
linea = 1

print ("{",end='')
for i in range(np.size(s_sen_enteros)):
    if i < ((linea * cant_por_linea) - 1):
        print (str(s_sen_enteros[i]) + ",",end='')
    else:
        if i == (np.size(s_sen_enteros) - 1):
            print (str(s_sen_enteros[i]),end='')
        else:                
            print (str(s_sen_enteros[i]) + ",\n",end='')
            linea += 1
        
print ("};")

