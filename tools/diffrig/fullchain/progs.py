"""Programs for the spp3 full-chain safety scan (stereo float32, 48 kHz).
Synthetic except tokyo (the loudness A/B clip, rig-only). Deterministic."""
import numpy as np, json, os, sys
SR = 48000
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
rng = np.random.default_rng(1106)
def save(name, x, meta=None):
    x.astype('<f4').tofile(f'{out}/{name}.f32')
    json.dump(meta or {}, open(f'{out}/{name}.json', 'w'))
# stepped sines, 1/3 octave 20 Hz-20 kHz, -20/-6/0 dBFS, 0.5 s + 0.1 s gap, L=R
F = [float(round(1000 * 2 ** (k / 3), 2)) for k in range(-17, 14)]
segs, x, pos = [], [], 0
for lv in (-20, -6, 0):
    for f in F:
        n = np.arange(int(0.5 * SR)); s = 10 ** (lv / 20) * np.sin(2 * np.pi * f * n / SR)
        x += [np.c_[s, s], np.zeros((int(0.1 * SR), 2))]
        segs.append({'f': f, 'level': lv, 'start': pos, 'len': len(n)}); pos += len(n) + int(0.1 * SR)
save('steps', np.vstack(x), {'segments': segs})
# tone bursts (transients): 40/80/160 Hz, 0 dBFS, 50 ms on / 450 ms off, 4 s each
x = []
for f in (40, 80, 160):
    n = np.arange(4 * SR); s = np.sin(2 * np.pi * f * n / SR) * ((n % (SR // 2)) < SR // 20)
    x.append(np.c_[s, s])
save('bursts', np.vstack(x), {'freqs': [40, 80, 160], 'seg_s': 4})
# pink noise, partly correlated stereo; -20 dBFS (10 s), -6 dBFS (60 s, thermal)
def pink(N):
    w = rng.standard_normal(N); W = np.fft.rfft(w); f = np.fft.rfftfreq(N, 1 / SR); f[0] = 1
    return np.fft.irfft(W / np.sqrt(f), N)
for lv, secs in ((-20, 10), (-6, 60)):
    N = secs * SR; a, b = pink(N), pink(N); p = np.c_[a + 0.5 * b, a - 0.5 * b]; p /= np.sqrt(np.mean(p ** 2))
    save(f'pink{lv}', p * 10 ** (lv / 20))
# music: tokyo clip at 0 and -12 dB
from scipy.io import wavfile
fs_, m = wavfile.read('/Volumes/AuroraS500/traces/audio/20261104-speaker-loudness/linux/tokyo.wav')
m = m.astype(float)[:30 * SR]
for g in (0, -12):
    save(f'tokyo{g}', m * 10 ** (g / 20))
print('ok', sorted(os.listdir(out)))
