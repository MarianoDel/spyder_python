## script para traer tickers desde yahoo!

## sigue a:
#https://ntguardian.wordpress.com/2016/09/19/introduction-stock-market-data-python-1/
#https://github.com/wilsonfreitas/awesome-quant
#https://github.com/pydata/pandas-datareader

#### ver tambien
# https://www.learndatasci.com/tutorials/python-finance-part-2-intro-quantitative-trading-strategies/
# https://www.learndatasci.com/tutorials/python-finance-part-yahoo-finance-api-pandas-matplotlib/

#### para matplotlib
# http://pbpython.com/effective-matplotlib.html


#from pandas.io import data, wb # becomes	ESTO NO VA MAS
from pandas_datareader import data, wb
import datetime
import pandas as pd

import pandas_datareader as pdr

import sys
if len(sys.argv) > 1:
    print(sys.argv[1])

#pdr.get_data_yahoo('AAPL')


# We will look at stock prices over the past year, starting at January 1, 2016
start = datetime.datetime(2017,7,5)
end = datetime.date.today()
tdelta = datetime.timedelta(days=2)

# Let's get Apple stock data; Apple's ticker symbol is AAPL
# First argument is the series we want, second is the source ("yahoo" for Yahoo! Finance), third is the start date, fourth is the end date
#apple = pdr.get_data_yahoo("AAPL", start, end)
#apple = pdr.get_data_yahoo('AAPL', start, end)
# print start
# print type (start)
# print start.date()

#me fijo si tengo lo pedido en el archivo
#get_info = True
get_info = False

try:
	backup = pd.read_csv("apple_csv.txt")

	for date_pd in backup['Date']:
		print date_pd
		print type (date_pd)
		start2 = start - tdelta
		if date_pd == str(start2.date()):
			#tengo la info no pido archivo
			print "ya tengo la info"
			get_info = False
			break

except Exception as e:
	print  "<p>Error: %s</p>" % str(e)


if (get_info):
	apple = pdr.get_data_yahoo('AAPL', start, end)
	print type(apple)
	print apple.head()
	# guardo info en archivo txt
	apple.to_csv ("apple_csv.txt")
