import numpy as np
import pandas as pd
import matplotlib as plt
import pandas.io.data as web
import math

DAX = web.DataReader(name = '^GDAXI', data_source = 'yahoo', start='2000-1-1')

DAX.info()

print DAX.tail()
dplot = DAX['Close']
#print dplot.tail()
plt.pyplot.plot(dplot)
plt.pyplot.show()

#los DataFrame de pandas son mutable entonces les puedo agregar nuevas columnas en forma dinamica
DAX['42d'] = pd.rolling_mean(DAX['Close'], window = 42)
DAX['252d'] = pd.rolling_mean(DAX['Close'], window = 252)
DAX['Return'] = np.log(DAX['Close'] / DAX['Close'].shift(1))    #divido los precios de cierre con los del dia anterior y LOGN

print DAX[['Close', 'Return', '42d', '252d']].tail()

#agrego los MAverage al dibujo
plt.pyplot.plot(DAX['42d'])
plt.pyplot.plot(DAX['252d'])
plt.pyplot.show()


DAX['Max_Vol'] = pd.rolling_std(DAX['Return'], window = 252) * math.sqrt(252)

plt.pyplot.figure(2)
plt.pyplot.subplot(311)
plt.pyplot.plot(DAX['Close'])
plt.pyplot.subplot(312)
plt.pyplot.plot(DAX['Return'])
plt.pyplot.subplot(313)
plt.pyplot.plot(DAX['Max_Vol'])
plt.pyplot.show()