import numpy as np
import matplotlib.pyplot as plt

# Открываем файл для чтения
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
        
# Инициализируем список для хранения данных


# fig, axs = plt.subplots(2, 1, layout='constrained')
plt.figure(1)
# axs[1].plot(count, np.abs(data),  color='grey')  # Используем scatter для диаграммы созвездия
plt.plot(count,(imag),color='red')  # Используем scatter для диаграммы созвездия
plt.plot(count,(real), color='blue')  # Используем scatter для диаграммы созвездия
plt.show()