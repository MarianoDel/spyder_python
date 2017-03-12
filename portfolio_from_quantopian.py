# -*- coding: utf-8 -*-
"""
Created on Sun Oct 30 16:23:08 2016

@author: med
"""

##QUANTOPIAN NUMPY LECTURE

import numpy as np
import matplotlib.pyplot as plt

N = 10
assets = np.zeros((N, 100))
returns = np.zeros((N, 100))

#Now we will simulate a base asset. We want the universe of stocks
#to be correlated with each other so we will use this initial value 
#to generate the others.
R_1 = np.random.normal(1.01, 0.03, 100)     #mean = 1.01 std=0.03
returns[0] = R_1
assets[0] = np.cumprod(R_1)     #el precio actual como producto acumulado de los retornos

# Generate assets that are correlated with R_1
for i in range(1, N):
    R_i = R_1 + np.random.normal(0.001, 0.02, 100)
    returns[i] = R_i # Set each row of returns equal to the new R_i array
    assets[i] = np.cumprod(R_i)
    
mean_returns = [(np.mean(R) - 1)*100 for R in returns]
return_volatilities = [np.std(R) for R in returns]

print 'MEAN RET:'
print mean_returns

plt.bar(np.arange(len(mean_returns)), mean_returns)
plt.xlabel('Stock')
plt.ylabel('Returns')
plt.title('Returns for {0} Random Assets'.format(N));

#agrego peso de cada accion en el portafolio
weights = np.random.uniform(0, 1, N)
weights = weights/np.sum(weights)

p_returns = np.dot(weights, mean_returns)
print "Expected return of the portfolio: ", p_returns

cov_mat = np.cov(returns)
print cov_mat

# INSERT VARIANCES AND COVARIANCES HERE
var_p = np.dot(np.dot(weights, cov_mat), weights.T)
vol_p = np.sqrt(var_p)
print "Portfolio volatility: ", vol_p

import sharpe

print "Sharpe Ratio: ", sharpe.sharpe(returns[1], 0.03, np.size(returns[1]))

