# -*- coding: utf-8 -*-
"""
Created on Sun Oct 30 17:31:46 2016

@author: med
"""

## Computing Volatility

# Load the required modules and packages
import numpy as np
import pandas as pd
import pandas.io.data as web

# Pull NIFTY data from Yahoo finance 
#NIFTY = web.DataReader('^NSEI',data_source='yahoo',start='6/1/2012', end='6/1/2016')
NIFTY = web.DataReader('WFC',data_source='yahoo',start='1/1/2015', end='today')

# Compute the logarithmic returns using the Closing price 
NIFTY['Log_Ret'] = np.log(NIFTY['Close'] / NIFTY['Close'].shift(1))

# Compute Volatility using the pandas rolling standard deviation function
NIFTY['Volatility'] = pd.rolling_std(NIFTY['Log_Ret'], window=252) * np.sqrt(252)
print(NIFTY.tail(15))

# Plot the NIFTY Price series and the Volatility
NIFTY[['Close', 'Volatility']].plot(subplots=True, color='blue',figsize=(8, 6))