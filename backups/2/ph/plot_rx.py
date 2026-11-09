import numpy as np
import matplotlib.pyplot as plt

name = "build/txdata.pcm"

data = []
imag = []
real = []
count = []
counter = 0
with open(name, "rb") as f:
    index = 0
    while (byte := f.read(2)):
        if(index %2 == 0):
            real.append(int.from_bytes(byte, byteorder='little', signed=True))
            counter += 1
            count.append(counter)
        else:
            imag.append(int.from_bytes(byte, byteorder='little', signed=True))
        index += 1
        

plt.figure(1)
plt.plot(count,(imag),color='red')
plt.plot(count,(real), color='blue')
plt.show()