# -*- coding: utf-8 -*-
"""
Created on Mon Nov 21 11:05:23 2016

@author: med
"""

##METAB
import matplotlib.pyplot as plt
import numpy as np

primeras = 3
segundas = 19
cantidad_pcb = primeras + segundas

##FALLAS ENCONTRADAS TOTALES
armado = 3
umal = 13
manipuleo = 6

##FALLAS ENCONTRADAS EN LAS QUE SALIERON
total_salidas = primeras + 11
salida_armado = 2
salida_umal = 8
salida_manipuleo = 4

lista_fallas = ["Armado" , "Manipuleo", "uMal"]
freq_fallas = [armado, manipuleo,umal]
freq_fallas_salida = [salida_armado, salida_manipuleo, salida_umal]

the_sum = sum(freq_fallas) # ie, 32 + 22 + 15 + 5 + 2
the_cumsum = np.cumsum(freq_fallas) #  32, 32 + 22, 32 + 22 + 15, 32 + 22 + 15 + 5, 32 + 22, + 15 + 5 + 2
#print 'fallas totales' + the_sum

fig = plt.figure(1)
ax1 = fig.add_subplot(111) # and a subplot
ax2 = ax1.twinx() # create a duplicate y axis

ax1.set_ylim(ymax=the_sum) 
ax1.bar([1,2,3], freq_fallas, color='b', label="Detectados en fabrica")
ax1.bar([1,2,3], freq_fallas_salida, color='y',label="Detectados en cliente")
#ax1.legend(prop={'size': 10}, loc='upper left')
ax1.legend(loc='upper left')

ax2.set_ylim(ymax=100) # %100

x = [1.4,2.4,3.4]
line, = ax1.plot(x, the_cumsum, 'ro-') # draw the  line
#plt.xticks([1,2,3], lista_fallas, rotation='vertical')
plt.xticks(x, lista_fallas)

ax1.set_ylabel('Defectos') # create the left y axis label
ax2.set_ylabel('Porcentaje') # create the right y axis label

plt.show()




#uMAL revisar version
#uMAL revisar temp ola
