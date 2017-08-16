## script para traer tickers desde yahoo!
from pandas_datareader import data, wb
import datetime
import pandas as pd
import pandas_datareader as pdr
import os


## Leo los tickers que quiero desde el archivo
def get_tickers ():
	""" reviso el archivo de los tickers (tickers.csv)
	"""
	try:
		tickers_file = pd.read_csv("tickers.csv",',')

	except Exception as e:
		print  "<p>Error on tickers file: %s</p>" % str(e)

	# print tickers_file
	# print type (tickers_file)
	# print tickers_file.shape
	# print tickers_file ["#Tickers"]

	return tickers_file

## Reviso si existe el archivo en el sistema
def check_tickers_file (t_name):
	""" reviso el sistema archivos segun los tickers
	"""
	ls = os.listdir('.')

	for l in ls:
		t = l.partition('.')
		t1 = str(t[0])
		#print t1 + " " + label
		if t_name == t1:
			#print "coincide archivo"
			return True

	return False

## creo nuevo archivo de tickers o hago update en los que ya existen
def update_all_tickers (df):
	"""reviso si existe archivo
	"""
	for d in df["#Tickers"]:
		if check_tickers_file(d):
			print "Archivo " + d + " encontrado"
			update_single_ticker(d)
		else:
			print "No existe " + d + " creo nuevo archivo"
			create_single_ticker(d)


## hago update del ticker que me pidan en el archivo
def update_single_ticker (t_name):
	"""ya se que existe el archivo, hago update hasta hoy
	"""

	end = datetime.date.today()
	backup_fail = False
	try:
		backup_df = pd.read_csv(t_name + ".csv")

	except Exception as e:
		print  "<p>Error reading csv file: %s</p>" % str(e)
		backup_fail = True

	if not backup_fail:
		last_date = backup_df.tail(1)
		last_date_series = last_date['Date'].iloc[0]
		print "last saved data " + last_date_series

		year, month, day = last_date_series.rsplit('-', 2)

		if last_date_series != str(end):
			start = datetime.datetime(int(year), int(month), int(day))
			# start = datetime.datetime(2017,7,5)

			tdelta = datetime.timedelta(days=1)
			start = start + tdelta

			tdf = pdr.get_data_yahoo(t_name, start, end)
			backup_df.append(tdf)
			print backup_df.tail()
			backup_df.to_csv (t_name + ".csv")
		else:
			pass

	else:
		#archivo corrompido creo un nuevo archivo
		create_single_ticker(t_name)
			




## creo el nuevo archivo del ticker
def create_single_ticker (t_name):
	"""no existe el archivo, lo creo hasta hoy
	"""
	start = datetime.datetime(2017,7,5)
	end = datetime.date.today()
	tdf = pdr.get_data_yahoo(t_name, start, end)
	print tdf.tail()
	# guardo info en archivo csv
	tdf.to_csv (t_name + ".csv")



# # import sys
# # if len(sys.argv) > 1:
# #     print(sys.argv[1])
#
# #pdr.get_data_yahoo('AAPL')
#
#
# # We will look at stock prices over the past year, starting at January 1, 2016
# start = datetime.datetime(2017,7,5)
# end = datetime.date.today()
# tdelta = datetime.timedelta(days=2)
#
# # Let's get Apple stock data; Apple's ticker symbol is AAPL
# # First argument is the series we want, second is the source ("yahoo" for Yahoo! Finance), third is the start date, fourth is the end date
# #apple = pdr.get_data_yahoo("AAPL", start, end)
# #apple = pdr.get_data_yahoo('AAPL', start, end)
# # print start
# # print type (start)
# # print start.date()
#
# #me fijo si tengo lo pedido en el archivo
# #get_info = True
# get_info = False
#
# try:
# 	backup = pd.read_csv("apple_csv.txt")
#
# 	for date_pd in backup['Date']:
# 		print date_pd
# 		print type (date_pd)
# 		start2 = start - tdelta
# 		if date_pd == str(start2.date()):
# 			#tengo la info no pido archivo
# 			print "ya tengo la info"
# 			get_info = False
# 			break
#
# except Exception as e:
# 	print  "<p>Error: %s</p>" % str(e)
#
#
# if (get_info):
# 	apple = pdr.get_data_yahoo('AAPL', start, end)
# 	print type(apple)
# 	print apple.head()
# 	# guardo info en archivo txt
# 	apple.to_csv ("apple_csv.txt")

df = get_tickers()
print df
update_all_tickers(df)
