import sympy

from sympy import *

sympy.init_printing()

s = Symbol('s')

G1 = 1 / (s + 1)
print(G1)

G2 = 10*s / (s + 10)
print(G2)

G=G1*G2
print(G)
print(G.simplify())

### reemplazo de variables utilizar funcion subs
y = Symbol('y')
print (G2)
G3 = G2.subs(s,y)
print (G3)

### evaluar la funcion en un punto, utilizo funcion subs
G4 = G3.subs(y, 1)
print (G4)

### evaluar como punto flotante un resultado de sympy
G5 = G4.evalf()
print (G5)
