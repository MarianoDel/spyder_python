from sympy import *

x, y, z = symbols('x y z')
init_printing(use_unicode=True)

expr1 = diff(cos(x), x)
expr2 = diff(exp(x**2), x)

print(expr1)
print(expr2)