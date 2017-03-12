# -*- coding: utf-8 -*-
"""
Created on Sun Nov 20 16:43:34 2016

@author: med
"""

import random
import matplotlib.pyplot as plt

N = 10
x_vals = range(N)
bit_vals = [-1, 1]
my_bits = []
for x in xrange(N):
    my_bits.append(bit_vals[random.randint(0, 1)])
print my_bits

fig = plt.figure()
ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
ax.set_ylim(ymin=-2, ymax=2)
ax.step(x_vals, my_bits, color='g')
ax.grid()
ax.set_yticks((-1, 1))

plt.show()