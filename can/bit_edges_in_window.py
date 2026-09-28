"""Every rising edge of every car-side CAN bit inside a time window of a route.

Scans src 0 (car bus), 128 (camera frames forwarded to the car) and 130 (car frames forwarded to
the camera). Lists openpilot alert sounds in the window, then low-activity bits only (1 to 12
rises and at most 30 flips) as (t+s, km/h, lateral active). Bit 0 is the most significant bit of
byte 0.

Usage: python bit_edges_in_window.py <route_id> <start_s> <end_s>
"""
import collections, os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
prefix, w0, w1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
t0 = None
prev, rises, flips_all = {}, collections.defaultdict(list), collections.Counter()
v, lat, sounds = 0.0, False, []
first_seg = max(0, int(w0 // 60) - 1)  # segments are 60 s; start one early so each frame has a previous value
for s in segs:
  p = os.path.join(BASE, s, "rlog.zst")
  idx = int(s.rsplit("--", 1)[1])
  for m in LogReader(p):
    t = m.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    if idx < first_seg:
      break
    rel = t - t0
    w = m.which()
    if w == "carState":
      v = m.carState.vEgo * 3.6
    elif w == "carControl":
      lat = m.carControl.latActive
    elif w == "selfdriveState" and w0 <= rel <= w1:
      sounds.append((round(rel, 1), str(m.selfdriveState.alertSound)))
    elif w == "can":
      for c in m.can:
        if c.src not in (0, 128, 130):
          continue
        key = (c.src, c.address)
        val = int.from_bytes(c.dat, "big"); n = len(c.dat) * 8
        if key in prev:
          d = prev[key] ^ val
          while d:
            b = d & -d; i = n - b.bit_length()
            if w0 <= rel <= w1:
              flips_all[(key, i)] += 1
              if val & b:
                rises[(key, i)].append((round(rel, 1), round(v), int(lat)))
            d ^= b
        prev[key] = val
print(f"window t+{w0:.0f}..{w1:.0f}s")
last = None
for t, s in sounds:
  if s != last:
    print(f"  sound t+{t} {s}"); last = s
rows = [(k, r) for k, r in rises.items() if 1 <= len(r) <= 12 and flips_all[k] <= 30]
rows.sort(key=lambda kr: (kr[0][0][1], kr[0][1]))
for (key, i), r in rows:
  print(f"src{key[0]:3d} 0x{key[1]:03X} bit{i:2d} rises {len(r):2d}: {r}")
