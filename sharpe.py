# -*- coding: utf-8 -*-
"""
Created on Sun Oct 30 17:35:14 2016

@author: med
"""

# Sharpe Ratio 
import numpy as np

#returns of the asset
#rf risk free
def sharpe(returns, rf, days=252):
    volatility = returns.std() * np.sqrt(days) 
    sharpe_ratio = (returns.mean() - rf) / volatility
    return sharpe_ratio
 

#The information ratio is an extension of the Sharpe ratio which replaces the risk-free rate of return 
#with the returns of a benchmark portfolio. It measures a trader’s ability to generate excess returns relative to a benchmark.
def information_ratio(returns, benchmark_returns, days=252):
    return_difference = returns - benchmark_returns 
    volatility = return_difference.std() * np.sqrt(days) 
    information_ratio = return_difference.mean() / volatility
    return information_ratio
    
    
# Modigliani Ratio
def modigliani_ratio(returns, benchmark_returns, rf, days=252):
    volatility = returns.std() * np.sqrt(days) 
    sharpe_ratio = (returns.mean() - rf) / volatility 
    benchmark_volatility = benchmark_returns.std() * np.sqrt(days)
    m2_ratio = (sharpe_ratio * benchmark_volatility) + rf
    return m2_ratio