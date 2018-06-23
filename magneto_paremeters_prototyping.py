# -*- coding: utf-8 -*-
"""
Created on Fri Jul  7 09:28:21 2017

@author: med
"""
import math

#ELEGIR LOS PARAMETROS DE ANTENNA Y MAXIMA CORRIENTE
#Resistance, Inductance, Current limit
# resistance = 48.2
# inductance = 400e-3
# current_limit = 0.887

resistance = 3.5
inductance = 19.e-3
current_limit = 3.5

# resistance = 10
# inductance = 78.2e-3
# current_limit = 2

#ELEGIR LOS PARAMETROS DE LA SEÑAL (seria del 100%, de otra manera bajar current_limit)
### el magneto actualmente manda lo siguiente:
    # def Envio_triangular_10 (self):
    #         self.s.Write("signal,070,070,0000,0049,0001,0001,0049,0000,0000,1\r\n")

    # def Envio_triangular_30 (self):
    #         self.s.Write("signal,070,070,0000,0016,0001,0001,0015,0000,0000,1\r\n")

    # def Envio_triangular_60 (self):
    #         self.s.Write("signal,070,070,0000,0007,0001,0001,0007,0000,0000,1\r\n")

    # def Envio_cuadrada_10 (self):
    #         self.s.Write("signal,070,070,0000,0001,0049,0001,0049,0000,0000,1\r\n")

    # def Envio_cuadrada_30 (self):
    #         self.s.Write("signal,070,070,0000,0001,0016,0001,0015,0000,0000,1\r\n")

    # def Envio_cuadrada_60 (self):
    #         self.s.Write("signal,070,070,0000,0001,0007,0001,0007,0000,0000,1\r\n")
            
rising_time = 1e-3
maintenance_time = 7e-3
falling_time = 1e-3
stop_time = 7e-3
period = rising_time + maintenance_time + falling_time + stop_time

#PARAMETROS FIJOS EN LA PLACA DE HARDWARE DEFAULT en V1.0
resistance_discharge = 1175.0
capacitance_discharge = 100e-6		#Farads
tau_discharge = 0.1175 				#RC
FUENTE_40V = 40.0					#40V y 200V son lo que esta hardcodeado
FUENTE_200V = 200.0
RSNUBBER_POWER_MAX = 15.0

#Calculo el valor en bits maximo en el ADC
#tengo 196mV / A + offset 456mV
peak_c = (current_limit * 1.5) * 0.196 + 0.46       #convierto corriente max a tensi�n con 50% de margen
peak_c_adc = (peak_c) / 3.0 * 4095                  #valor pico permitido en ADC
peak_c_adc = round(peak_c_adc, 0)

print "T = " + str(period) + "s, freq = " + str(round(1/period,2)) + "Hz"
print "Corriente pico en V a la entrada del micro = " + str(peak_c) + "V"
print "Valor maximo del registro del ADC en el micro = " + str(peak_c_adc)
print ""

#Calculo etapa Rising de la senial
print "RISING"
voltage = (current_limit * inductance / rising_time) * 2.0
voltage2 = current_limit * resistance

if (voltage > voltage2):
	#subida rapida
	print "Fast Rise"
	#ajusto voltage
	#voltage = voltage - voltage2 / 2.0
	print "rising_time elegido: " + str(rising_time) + "s"
	print "Tension inicial V1 = " + str(round(voltage, 1)) + "V"

	print "Tension final V2 = " + str(round(voltage2, 1)) + "V"
	if (voltage < FUENTE_40V):
		pwm_40_1 = voltage / FUENTE_40V
		pwm_40_2 = voltage2 / FUENTE_40V
		print "Utiliza PWM_40 V1: duty: " + str(round(pwm_40_1, 3))
		print "Utiliza PWM_40 V2: duty: " + str(round(pwm_40_2, 3))
	elif voltage < FUENTE_200V:
		pwm_200_1 = voltage / FUENTE_200V
		pwm_200_2 = voltage2 / FUENTE_200V
		print "Utiliza PWM_200 V1 duty: " + str(round(pwm_200_1, 3))
		print "Utiliza PWM_200 V2 duty: " + str(round(pwm_200_2, 3))
	else:
		print "ERROR supera el limite de driveo permitido!!!"
else:
	#subida lenta
	print "Slow Rise"
	print "rising_time elegido: " + str(rising_time) + "s"
	if (voltage2 < FUENTE_40V):
		pwm_40_1 = 0.0
		pwm_40_2 = voltage2 / FUENTE_40V
		print "Utiliza PWM_40 V1: duty: " + str(round(pwm_40_1, 3))
		print "Utiliza PWM_40 V2: duty: " + str(round(pwm_40_2, 3))
	else:
		print "ERROR supera el limite de driveo permitido!!!"


print ""

#Calculo etapa Maintenance de la senial
print "MAINTENANCE"
voltage = resistance * current_limit
print "maintenance_time elegido: " + str(maintenance_time) + "s"
print "Tension inicial/final V1 = " + str(round(voltage, 1)) + "V"
if (voltage < FUENTE_40V):
	pwm_40 = voltage / FUENTE_40V
	print "Utiliza PWM_40 V1 duty: " + str(round(pwm_40, 3))
else:
	print "ERROR supera el limite de driveo permitido!!!"

print ""

#Calculo etapa Falling de la senial
print "FALLING"
print "Tiempo de caida pedido: " + str(round(falling_time,4))
LR_tau = inductance / resistance
print "Tau = " + str(round(LR_tau,4)) + "s"

#tension en snubber network por balance de energia PRsnubber = PL - PRl
PL = 0.5 * inductance * current_limit * current_limit / period
print "L power = " + str(PL) + "W"
voltageRL = current_limit * resistance * math.sqrt(falling_time / (3.0 * period))
print "voltage RL = " + str(voltageRL) + "V"
PRL = voltageRL * voltageRL / resistance
PRsnubber = PL - PRL
print "Rsnubber power = " + str(PRsnubber) + "W"
if PRsnubber > RSNUBBER_POWER_MAX:
	print "ERROR supera el limite de potencia permitido!!!"

Vsnubber = math.sqrt(PRsnubber * resistance_discharge)
print "voltage snubber = " + str(Vsnubber) + "V"

#ajusto Vsnubber para calcular descarga rapida
Vsnubber2 = Vsnubber + current_limit * resistance
Td = (-LR_tau) * math.log(1 - (current_limit * resistance) / Vsnubber2)

if Td > (falling_time + stop_time):
	print "ERROR supera el limite de tiempo permitido!!!"

Td_aux = Td + 0.25e-3		#sumo un margen por variaciones de hard

print "Vsnubber = " + str(round(Vsnubber,1)) + "V; Tiempo de descarga rapida = " + str(round(Td, 5)) + "s"
if (falling_time > LR_tau):
	print "Utilizo descarga acompañando LR Tau"
elif (falling_time > (1.2 * Td_aux)):
	print "Utilizo descarga LR Tau y acelero al final con desc. rapida"
else:
	print "Utilizo descarga rapida"
