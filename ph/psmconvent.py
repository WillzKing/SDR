# mp3_to_pcm.py
import librosa
import numpy as np

mp3_file = "audio.mp3"
pcm_file = "input.pcm"

# Загружаем как моно, 44100 Гц, float32 в [-1.0, 1.0]
y, sr = librosa.load(mp3_file, sr=44100, mono=True)

# float32 → int16
pcm_data = (y * 32767).astype(np.int16)

# Пишем сырые PCM-данные (без заголовка)
pcm_data.tofile(pcm_file)

print(f"Saved {len(pcm_data)} samples, sr={sr} → {pcm_file}")