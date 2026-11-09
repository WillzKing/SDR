from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

file = Path(__file__).resolve().parent / 'txdata.pcm'
data = np.fromfile(file, dtype='<i2')
if not data.size or data.size % 2:
    raise ValueError('Сначала запусти TX, чтобы создать полный буфер')
I = data[0::2]
Q = data[1::2]
plt.plot(I[:200], color='blue', label='I')
plt.plot(Q[:200], color='red', label='Q')
plt.xlabel('Номер отсчёта TX')
plt.ylabel('Значение int16')
plt.legend()
plt.grid()
plt.tight_layout()
plt.savefig(file.parent / 'tx_plot.png', dpi=150)
plt.show()
