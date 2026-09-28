"""Extract a route's microphone audio to WAV and list tonal beeps with driving context.

Usage on device:
  PYTHONPATH=/data/openpilot:/data/openpilot/openpilot /usr/local/venv/bin/python /data/beeps.py <route_prefix> <out.wav>
"""
import os
import sys
import wave
import warnings

import numpy as np

warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
FRAME = 0.05
BAND = (300.0, 5000.0)
TONAL_RATIO = 12.0   # peak bin power over the band median
MIN_DB = -55.0       # absolute floor so silence is not tonal
MIN_DUR = 0.08


def main():
  prefix, out = sys.argv[1], sys.argv[2]
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  t0, sr = None, None
  chunks = []
  ctx = []  # (t, v, lat, sound)
  v, lat, snd = 0.0, 0, "none"
  for s in segs:
    p = os.path.join(BASE, s, "rlog.zst")
    if not os.path.exists(p):
      continue
    for m in LogReader(p):
      w = m.which()
      t = m.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      if w == "rawAudioData":
        sr = m.rawAudioData.sampleRate
        chunks.append((t - t0, np.frombuffer(m.rawAudioData.data, dtype=np.int16)))
      elif w == "carState":
        v = m.carState.vEgo * 3.6
      elif w == "carControl":
        lat = int(m.carControl.latActive)
      elif w == "selfdriveState":
        snd = str(m.selfdriveState.alertSound)
        ctx.append((t - t0, v, lat, snd))

  # place chunks on a timeline so gaps stay silent and times stay true
  end = chunks[-1][0] + len(chunks[-1][1]) / sr
  audio = np.zeros(int(end * sr) + sr, dtype=np.int16)
  for t, a in chunks:
    i = int(t * sr)
    audio[i:i + len(a)] = a[:len(audio) - i]
  with wave.open(out, "wb") as f:
    f.setnchannels(1)
    f.setsampwidth(2)
    f.setframerate(sr)
    f.writeframes(audio.tobytes())

  n = int(FRAME * sr)
  win = np.hanning(n)
  freqs = np.fft.rfftfreq(n, 1 / sr)
  band = (freqs >= BAND[0]) & (freqs <= BAND[1])
  tonal = []
  for k in range(len(audio) // n):
    x = audio[k * n:(k + 1) * n].astype(np.float32) / 32768.0
    spec = np.abs(np.fft.rfft(x * win)) ** 2
    b = spec[band]
    peak = b.max()
    db = 10 * np.log10(peak + 1e-12)
    ratio = peak / (np.median(b) + 1e-12)
    tonal.append((k * FRAME, ratio > TONAL_RATIO and db > MIN_DB, freqs[band][b.argmax()], db))

  events, cur = [], None
  for t, on, f, db in tonal:
    if on and (cur is None or abs(f - cur["f"][-1]) < 150):
      if cur is None:
        cur = {"t": t, "f": [], "db": []}
      cur["f"].append(f)
      cur["db"].append(db)
      cur["end"] = t + FRAME
    elif cur is not None:
      if cur["end"] - cur["t"] >= MIN_DUR:
        events.append(cur)
      cur = None if not on else {"t": t, "f": [f], "db": [db], "end": t + FRAME}

  def context(t):
    best = min(ctx, key=lambda c: abs(c[0] - t)) if ctx else (t, 0, 0, "none")
    op = any(c[3] != "none" and abs(c[0] - t) < 1.5 for c in ctx)
    return best[1], best[2], op

  print(f"{prefix}: {len(segs)} segs, {end:.0f} s of audio at {sr} Hz -> {out}")
  print(f"{len(events)} tonal events (>= {MIN_DUR * 1000:.0f} ms)")
  for e in events:
    v, la, op = context(e["t"])
    f = np.median(e["f"])
    print(f"  t+{e['t']:7.2f}s  dur {e['end'] - e['t']:5.2f}s  {f:6.0f} Hz  {max(e['db']):6.1f} dB  "
          f"{v:5.1f} km/h lat={la}{'  <- openpilot sound nearby' if op else ''}")


if __name__ == "__main__":
  main()
