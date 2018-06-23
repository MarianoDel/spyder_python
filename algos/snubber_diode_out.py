#utiliza python3
import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
from tc_udemm import sympy_to_lti
from math import sqrt, exp
from scipy.signal import step, step2


### parametros del circuito diodo de salida
#
Vin = 208        #tension que se le aplica al diodo (inversa que aca esta dada vuelta)
Lleak = 1e-6     #inducatancia parasita del secundario
Cij= 9e-12       #capacitancia interna del diodo (simulada o de hoja de datos, ver Vin porque se mueve)
RL = 127         #R de salida (carga)

s = sp.Symbol('s')

#Vout = Vin * (1 / ((Cij*Lleak) * (s**2 + s * (Cij * RL /(Cij * Lleak)) + 1 /(Cij * Lleak))))
#snubberless_system_s = 1 / ((Cij*Lleak) * (s**2 + s * (Cij * RL /(Cij * Lleak)) + 1 /(Cij * Lleak)))
wn2 = 1 /(Cij * Lleak)
two_psi_wn = Cij * RL /(Cij * Lleak)
k = 1 / (Cij*Lleak)
snubberless_system_s = k /(s**2 + s * two_psi_wn + wn2)
print ("snubberless_system_s:")
print (snubberless_system_s)
print ()

wn = sqrt(wn2)
fn = wn /(2*np.pi)
print ("wn y fn:")
print (wn)
print (fn)
print ()

psi = two_psi_wn /(2 * wn)
print ("psi:")
print (psi)
print ()

if psi < 1:
    overshoot = exp((-np.pi)*psi/(sqrt(1-psi**2)))
    print ("overshoot:")
    print (overshoot)
else:
    print ("no overshoot")
print ()
    

snubberless_system = sympy_to_lti(snubberless_system_s)

print (snubberless_system)
### respuesta escalon del diodo de salida
t = np.linspace(0, 200e-9, 10000)
# T, yout = step2(snubberless_system, T=t)
T, yout = step(snubberless_system, T=t)

fig, ax = plt.subplots()
ax.plot(T, yout, 'r-', linewidth=2, label=r'$y=\sin(x)$', alpha=0.6)
ax.legend(loc='upper right')
# ax.set_yticks([-1, 0, 1])
ax.set_title('Test plot')
plt.show()

### Cicuito con snubber
#
#
Cn = 100e-12    #capacitor de snubber
Rn = 100        #resistencia del snubber
#

### circuito reducido a segundo orden
wn2 = 1 /(Lleak * (Cn + Cij))
two_psi_wn = (RL*(Cn+Cij)+Cn*Rn)/(Lleak * (Cn + Cij))
k = 1 /(Lleak * (Cn + Cij))
zero = s * (Cn*Rn) + 1
snubber_system_s = k * zero /(s**2 + s * two_psi_wn + wn2)
print ("snubber_system_s:")
print (snubber_system_s)
print ()

wn = sqrt(wn2)
fn = wn /(2*np.pi)
print ("wn y fn:")
print (wn)
print (fn)
print ()

psi = two_psi_wn /(2 * wn)
print ("psi:")
print (psi)
print ()

if psi < 1:
    overshoot = exp((-np.pi)*psi/(sqrt(1-psi**2)))
    print ("overshoot:")
    print (overshoot)
else:
    print ("no overshoot")
print ()
    
snubber_system = sympy_to_lti(snubber_system_s)
print (snubber_system)
### respuesta escalon del diodo de salida
t = np.linspace(0, 200e-9, 10000)
# T, yout = step2(snubberless_system, T=t)
T, yout = step(snubber_system, T=t)

fig, ax = plt.subplots()
ax.plot(T, yout, 'r-', linewidth=2, label=r'$y=\sin(x)$', alpha=0.6)
ax.legend(loc='upper right')
# ax.set_yticks([-1, 0, 1])
ax.set_title('Test plot')
plt.show()


### Dejando Cn fijo y moviendo Rn
Cn = 100e-12
Rn = [10, 20, 40, 80, 160, 320]
#'b', 'g', 'r', 'c', 'm', 'y', 'k', 'w'
colors = ['r','b','g','c','m','y','k','w']

fig, ax = plt.subplots()
i = 0
for Rn in Rn:
    wn2 = 1 /(Lleak * (Cn + Cij))
    two_psi_wn = (RL*(Cn+Cij)+Cn*Rn)/(Lleak * (Cn + Cij))
    k = 1 /(Lleak * (Cn + Cij))
    zero = s * (Cn*Rn) + 1
    snubber_system_s = k * zero /(s**2 + s * two_psi_wn + wn2)

    snubber_system = sympy_to_lti(snubber_system_s)
    t = np.linspace(0, 200e-9, 10000)
    T, yout = step(snubber_system, T=t)

    # ax.plot(T, yout, colors[i], linewidth=2, label=str(colors[i])+str([Rn]), alpha=0.6)
    ax.plot(T, yout, colors[i], linewidth=2, label=str(Rn), alpha=0.6)
    i += 1


ax.legend(loc='upper right')
# ax.set_yticks([-1, 0, 1])
ax.set_title('Test plot')
plt.show()


### Dejando Rn fija y moviendo Cn
Rn = 40
Cn = [100e-12, 200e-12, 300e-12, 400e-12, 500e-12, 600e-12]

#'b', 'g', 'r', 'c', 'm', 'y', 'k', 'w'
colors = ['r','b','g','c','m','y','k','w']

fig, ax = plt.subplots()
i = 0
for Cn in Cn:
    wn2 = 1 /(Lleak * (Cn + Cij))
    two_psi_wn = (RL*(Cn+Cij)+Cn*Rn)/(Lleak * (Cn + Cij))
    k = 1 /(Lleak * (Cn + Cij))
    zero = s * (Cn*Rn) + 1
    snubber_system_s = k * zero /(s**2 + s * two_psi_wn + wn2)

    snubber_system = sympy_to_lti(snubber_system_s)
    t = np.linspace(0, 200e-9, 10000)
    T, yout = step(snubber_system, T=t)

    # ax.plot(T, yout, colors[i], linewidth=2, label=str(colors[i])+str([Rn]), alpha=0.6)
    ax.plot(T, yout, colors[i], linewidth=2, label=str(Cn), alpha=0.6)
    i += 1


ax.legend(loc='upper right')
# ax.set_yticks([-1, 0, 1])
ax.set_title('Test plot')
plt.show()


### Derivada respecto de Cn
# d_snubber_to_Cn = sp.diff(snubber_system_s, Cn)
