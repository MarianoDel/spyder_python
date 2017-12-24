## script para analisis de temperatura METAB
#utiliza archivo del datalogger "datalogger.txt" o nombre modificado

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import math


irms = np.asarray([0.32, 0.505, 0.730, 0.953])
vpp = np.asarray([1.12, 1.72, 2.44, 3.12])


###AJUSTO PUNTA EXTERNA (la de la plaqueta)
# linear = np.ones_like(vpp)
linear = irms * 3.24 + 0.071	#contra eje x int eje y ext


# for y in ntc_ext:
# 	print y

# for i in range(np.size(ntc_ext)):
# 	# if ntc_ext[i] > 734:
# 	# 	ntc_ext[i] = ntc_ext[i] * 1330.0 / 734.0
# 	# elif ntc_ext[i] > 716:
# 	# 	ntc_ext[i] = ntc_ext[i] * 1280.0 / 716.0
# 	# elif ntc_ext[i] > 676:
# 	# 	ntc_ext[i] = ntc_ext[i] * 1136.0 / 676.0
# 	# elif ntc_ext[i] > 626:
# 	# 	ntc_ext[i] = ntc_ext[i] * 987.0 / 626.0
# 	# else:
# 	# 	ntc_ext[i] = ntc_ext[i] * 735.0 / 563.0
# 	# ntc_ext[i] = ntc_ext[i] * 1330.0 / 744.0
# 	ntc_ext[i] = float(ntc_int[i]) / float(ntc_ext[i])



###GRAFICO LAS MEDIDAS
fig, ax = plt.subplots()
ax.set_title('Vpp = f(irms)')

ax.plot(irms, vpp, 'b')
ax.plot(irms, linear, 'g')

#Proxy Patches
# blue_patch = mpatches.Patch(color='blue', label='ntc_ext_adj')	#AJUSTADA
# red_patch = mpatches.Patch(color='red', label='ntc_int')
# green_patch = mpatches.Patch(color='green', label='6_C')
# cyan_patch = mpatches.Patch(color='cyan', label='8_C')
# yellow_patch = mpatches.Patch(color='yellow', label='10_C')
# black_patch = mpatches.Patch(color='black', label='12_C')
# plt.legend(handles=[blue_patch, red_patch, green_patch, cyan_patch, yellow_patch, black_patch],loc='lower right')

plt.show()

# try:
# 	df = pd.read_csv("logger_6bips_pd.txt")	#DataFrame de pandas
#
# 	# for date_pd in backup['Date']:
# 	# 	print date_pd
# 	# 	print type (date_pd)
# 	# 	start2 = start - tdelta
# 	# 	if date_pd == str(start2.date()):
# 	# 		#tengo la info no pido archivo
# 	# 		print "ya tengo la info"
# 	# 		get_info = False
# 	# 		break
# 	"""Le doy forma, quito la primera fila y la ultima columna"""
#
# 	# df.drop(df.index[0])
# 	#print df2.head(2)
# 	print '\n'
# 	print df.index[0]
# 	print df.index[1]
# 	print df.index[-1]
# 	print '\n'
# 	print df.head(3)
# 	print df.columns
#
# 	df.columns = ['SECS', 'NTC_OUT', 'NTC_IN']
# 	#df.columns = ['SECS']
#
# 	print df.columns
#
# 	# plt.figure()
# 	# df.plot()
# 	# plt.show()
#
#
# 	# print df2.head(2)
# 	# print type(df2)
# 	# print df2.columns
#
#
# except Exception as e:
# 	print  "<p>Error: %s</p>" % str(e)
