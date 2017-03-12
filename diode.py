# -*- coding: utf-8 -*-
"""
Created on Fri Mar 18 18:50:04 2016

@author: root
"""
import numpy as np
import matplotlib.pyplot as plt
#import math

#CURVA DEL DIODO
n = 1;
Vt = 0.025
Is = 10**-12
Vd1=np.arange(-5,0.6,0.01)
Id = Is*(np.exp(Vd1/(n*Vt))-1)
#Id = Is*((Vd1/(n*Vt))-1)

plt.plot(Vd1, Id)

#CORRIENTE EN EL CIRCUITO
Rserie = 1000
Vd2 = np.arange(-5,12,0.01)
Id2 = (12 - Vd2)/Rserie
plt.plot(Vd2,Id2)