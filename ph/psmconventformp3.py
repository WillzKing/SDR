# pcm_to_mp3.py
import numpy as np
from pydub import AudioSegment

pcm_file = "../build/rxdata.pcm"
mp3_file = "rxdata.mp3"

# Читаем сырые int16
pcm_data = np.fromfile(pcm_file, dtype=np.int16)

# Собираем аудио-объект
audio = AudioSegment(
    data=pcm_data.tobytes(),
    sample_width=2,     # 2 байта = int16
    frame_rate=44100,
    channels=1,         # моно
)

audio.export(mp3_file, format="mp3", bitrate="192k")
print(f"Saved {mp3_file}")