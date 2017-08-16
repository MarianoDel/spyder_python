import numpy as np
import matplotlib.pyplot as plt

#tension maxima del triangulo; inductancia; tiempo rise
U = 40.0
L = 18e-3
t_rise = 3e-3

t_graph = 10e-3

t = np.arange(0, t_graph, t_graph / 1000.0)

i_t = U / L * (t - t**2 / ( 2 * t_rise))

plt.plot(t, i_t)
plt.show()
