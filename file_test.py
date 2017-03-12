# -*- coding: utf-8 -*-
"""
Created on Sun Jul 24 18:47:04 2016

@author: med
"""

import numpy as np
f = open ("TrainExer21.txt")
#obs, wage, logwage, female, age, educ, partt = np.loadtxt(f, skiprows = 0)
data = np.loadtxt(f, skiprows = 1)

print (data)