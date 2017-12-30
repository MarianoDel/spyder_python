from sympy import *

x, y, z = symbols('x y z')
expr = x**2 + x*y

srepr(expr)

#"Add(Pow(Symbol('x'), Integer(2)), Mul(Symbol('x'), Symbol('y')))"

print(expr)