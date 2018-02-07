##PRUEBA MLCC CON LIBRARY http://practicalcryptography.com/resources/
##
## https://github.com/jameslyons/python_speech_features

from features import mfcc
from features import logfbank
import scipy.io.wavfile as wav
import matplotlib.pyplot as plt

#para abrir archivo con fordward slashes / o double backwards slashes \\
(rate,sig) = wav.read("C:/Users/Mariano/Documents/Proyectos/Python/num1.wav")

print (rate)

#figure(1)
#clf()# clear the figure - "#" is the comment symbol in Python

plt.plot(sig)
plt.show() #may not be necessary depending on how your graphics thread is running.
plt.draw()

