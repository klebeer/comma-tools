"""Rate and largest gaps of the camera's CRZ_CTRL (0x21C) frames on bus 2, per route.

The relay fails closed if the camera stops sending 0x21C for over 1 s, so the gaps show how much
margin it has. Gaps print as (milliseconds, seconds into the route).

Usage: python crz_ctrl_rate_and_gaps.py <route_id> [<route_id> ...]
"""
import os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
for prefix in sys.argv[1:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  last = first = None; gaps = []; n = 0
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      if m.which() != "can": continue
      t = m.logMonoTime / 1e9
      for f in m.can:
        if f.src == 2 and f.address == 0x21C:
          if last is not None: gaps.append((t - last, t - first))
          first = t if first is None else first
          last = t; n += 1
  g = sorted(gaps, reverse=True)[:3]
  print(f"{prefix}: {n} frames over {last - first:.0f}s, {n / (last - first):.1f} Hz, largest gaps {[(round(a*1000), round(b)) for a, b in g]} (ms, at t+s)")
