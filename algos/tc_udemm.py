# -*- coding: utf-8 -*-
#usar python3 o python?
import numpy as np
from scipy import signal
from sympy import *
from scipy.signal import lti, step, bode, zpk2tf



#### FUNCIONES TC-2017 ESTIGARRIBIA y SALARIATO ####
#Multiplico funciones
def multiplico_sistemas(sys1, sys2):

    if not isinstance(sys1, signal.lti):
        sys1 = signal.lti(*sys1)
    if not isinstance(sys2, signal.lti):
        sys2 = signal.lti(*sys2)
    num = np.polymul(sys1.num, sys2.num)
    den = np.polymul(sys1.den, sys2.den)
    sys = signal.lti(num, den)
    return sys

#Sumo funciones
def sumo_sistemas(sys3, sys4):

    if not isinstance(sys3, signal.lti):
        sys3 = signal.lti(*sys3)
    if not isinstance(sys4, signal.lti):
        sys4 = signal.lti(*sys4)
    num = np.polyadd(sys3.num, sys4.num)
    den = np.polyadd(sys3.den, sys4.den)
    sys = signal.lti(num, den)
    return sys

#Realimento con Multiple Fedback, hay que tener en cuenta cual es la
#realimentacion, tipicamente es 1
#si alpha es el lazo forward y beta es el lazo backwards... N -> numerador; D -> denominador
#alpha = Nalpha / Dalpha
#beta = Nbeta / Dbeta
#syst realimentado = (Nalpha * Dbeta) / (Dalpha * Dbeta + Nalpha * Nbeta)
def realimento (alpha, beta):

	if not isinstance(alpha, signal.lti):
	   alpha = signal.lti(*alpha)

	if not isinstance(beta, signal.lti):
	   beta = signal.lti(*beta)

	#numerador sistema
	num_sys = np.polymul(alpha.num, beta.den)

	#armo el denominador
	den_sys1 = np.polymul(alpha.den, beta.den)
	den_sys2 = np.polymul(alpha.num, beta.num)
	den_sys = np.polyadd(den_sys1, den_sys2)

	#convierto nuevamente a TF
	sys = signal.lti(num_sys, den_sys)
	return sys


### FUNCONES DE INTERNET ########
from scipy import signal
import sympy as sy

def lti_to_sympy(lsys, symplify=True):
	""" Convert Scipy's LTI instance to Sympy expression """
	s = sy.Symbol('s')
	G = sy.Poly(lsys.num, s) / sy.Poly(lsys.den, s)
	return sy.simplify(G) if symplify else G

def sympy_to_lti(xpr, s=sy.Symbol('s')):
	""" Convert Sympy transfer function polynomial to Scipy LTI """
	num, den = sy.simplify(xpr).as_numer_denom()  # expressions
	p_num_den = sy.poly(num, s), sy.poly(den, s)  # polynomials
	c_num_den = [sy.expand(p).all_coeffs() for p in p_num_den]  # coefficients
	l_num, l_den = [sy.lambdify((), c)() for c in c_num_den]  # convert to floats
	return signal.lti(l_num, l_den)

# #ejemplo de uso
# pG, pH, pGH, pIGH = sy.symbols("G, H, GH, IGH")  # only needed for displaying
#
#
# # Sample systems:
# lti_G = signal.lti([1], [1, 2])
# lti_H = signal.lti([2], [1, 0, 3])
#
# # convert to Sympy:
# Gs, Hs = lti_to_sympy(lti_G), lti_to_sympy(lti_H)
#
# print("Converted LTI expressions:")
# print (sy.Eq(pG, Gs))
# print (sy.Eq(pH, Hs))
#
# print("Multiplying Systems:")
# GHs = sy.simplify(Gs*Hs).expand()  # make sure polynomials are canceled and expanded
# print (sy.Eq(pGH, GHs))
#
#
# print("Closing the loop:")
# IGHs = sy.simplify(GHs / (1+GHs)).expand()
# print (sy.Eq(pIGH, IGHs))
#
# print("Back to LTI:")
# lti_IGH = sympy_to_lti(IGHs)
# print(lti_IGH)
