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
