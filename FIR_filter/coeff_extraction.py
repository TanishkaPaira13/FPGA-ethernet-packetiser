import matplotlib.pyplot as plt
import scipy.signal as sig
import numpy as np
from math import pi

#values
Fs = 125000000;
n= 63;
fc= 50000000;

#logic
w_c = 2*fc/Fs
taps = sig.firwin(n+1 ,w_c)
[w,h]= sig.freqz(taps,worN=2000)
w=Fs*w/(2*pi)

#conversion to dB
h_db=20*np.log10(abs(h))

#Plot
plt.figure(2)
plt.plot(w,h_db);plt.title('FIR filter response')
plt.xlabel('Frequency(Hz)');
plt.ylabel('Magnitude(dB)')
plt.grid('on')

#coefficients
coefficients = sig.firwin(n+ 1, w_c, window='hamming')
print("Filter Coefficients:")
for i, coeff in enumerate(coefficients):
    print(f"Tap {i}: {coeff}")

#quantization function
def float_to_q15(val):
    val = np.clip(val, -1.0, 0.999969482421875)
    return int(round(val * (2**15)))

#quantization
q15_coeffs = [float_to_q15(c) for c in coefficients]

print("Q1.15 Fixed-Point Coefficients:")
for i, val in enumerate(q15_coeffs):
    print(f"Tap {i}: {val}")
