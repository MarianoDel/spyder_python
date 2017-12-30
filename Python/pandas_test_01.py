import numpy as np
import pandas as pd
import matplotlib as plt



a = np.random.standard_normal((9, 4))
a.round(6)

df = pd.DataFrame(a)
print df

df.columns = [['pri', 'seg', 'ter', 'cua' ]]
print df

#vector de date time
dates = pd.date_range('2015-1-1', periods=9, freq = 'M')
print dates

#aplico las fechas al DataFrame
df.index = dates
print df

#df.cumsum().plot(style='r', lw=2.0)
df1 = df.cumsum()
plt.pyplot.plot(df1)
plt.pyplot.xlabel('date')
plt.pyplot.show()
