# usar python3
# ajuste de curva en el medidor de flujo magnetico

# el lcd muestra los valores en puntos ADC procesados y los convierto a campo
# el efecto hall es de 2.5mVp -> 1G

# TABLA de mediciones
# mVp  valor x    valor y    valor z
# 2       56       48         62
# 125    207      200        206
# 250    357      352        358
# 500    663      658        669


import numpy as np
import matplotlib.pyplot as plt
# import matplotlib.patches as mpatches
# import math


flux_lcd = np.asarray([56, 207, 357, 663])
mVp =  np.asarray([2, 125, 250, 500])
Gauss = mVp / 2.5

# ###AJUSTO LCD al flujo teorico
# flux_lcd_adj = flux_lcd * 0.33 - 18.5
flux_lcd_adj = flux_lcd - 56
flux_lcd_adj = flux_lcd_adj * 0.33
print (Gauss)
print (flux_lcd_adj)

# ###GRAFICO LAS MEDIDAS
fig, ax = plt.subplots()
ax.set_title('Blue: teorica; Red: ADC; Green: Ajustada')
#
ax.plot(mVp, Gauss, 'b')
ax.plot(mVp, flux_lcd, 'r')
ax.plot(mVp, flux_lcd_adj, 'g')

plt.show()
# #Proxy Patches
# # blue_patch = mpatches.Patch(color='blue', label='ntc_ext_adj')	#AJUSTADA
# # red_patch = mpatches.Patch(color='red', label='ntc_int')
# # green_patch = mpatches.Patch(color='green', label='6_C')
# # cyan_patch = mpatches.Patch(color='cyan', label='8_C')
# # yellow_patch = mpatches.Patch(color='yellow', label='10_C')
# # black_patch = mpatches.Patch(color='black', label='12_C')
# # plt.legend(handles=[blue_patch, red_patch, green_patch, cyan_patch, yellow_patch, black_patch],loc='lower right')


