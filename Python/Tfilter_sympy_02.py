#from scipy.signal import *
import scipy.signal as signal
import numpy as np


#convierto G1 a un sistema num den
num = [30600, 7727272727.27273]
den = [1.96e-6, 2.4304, 43327.2727272727, 7727272727.27273]


#w, h = freqs(b, a, worN=np.logspace(-1, 2, 1000))
w, h = signal.freqs(num, den)   #w esta en rad/s

import matplotlib.pyplot as plt
plt.semilogx((w/6.28), 20 * np.log10(abs(h)))
plt.xlabel('Frequency')
plt.ylabel('Amplitude response [dB]')
plt.grid()
plt.show()

T, yout = signal.step((num, den))
plt.figure(2)
plt.clf()
plt.plot(T, yout)
plt.show()

Ts = 1./25000
#sysd = signal.cont2discrete ((num, den), Ts)
#w1, h1 = signal.freqz(sysd)

numd, dend, dt = signal.cont2discrete ((num, den), Ts)
numd1 = numd[0,:]
w1, h1 = signal.freqz(numd1, dend, worN=None, whole=0, plot=None)


plt.figure(3)
plt.clf()
plt.plot(w1, 20 * np.log10(abs(h1)), 'b')
plt.ylabel('Amplitude [dB]', color='b')
plt.xlabel('Frequency [rad/sample]')
plt.show()

#u = np.ones(1000)
#t_in = np.arange(0, 1000)

#t_out, y = dlsim(sysd, u, t=t_in)

#tout, yout = signal.dstep((numd1, dend, Ts), x0=None, t=None, n=None)
tout, yout = signal.dstep((numd1, dend, Ts), x0=None, t=None, n=25)
#split del tuple
y_out = yout[0]
    
plt.figure(4)
plt.clf()
plt.plot (tout, y_out)
plt.show()
