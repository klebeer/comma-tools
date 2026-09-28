import os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
for prefix in sys.argv[1:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  t0, v, lat, prev, start, eps = None, 0.0, False, False, None, []
  for s in segs:
    p = os.path.join(BASE, s, "rlog.zst")
    if not os.path.exists(p):
      continue
    for m in LogReader(p):
      w = m.which()
      if w not in ("carState", "carControl"):
        continue
      t = m.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      if w == "carControl":
        lat = m.carControl.latActive
        continue
      cs = m.carState
      v = cs.vEgo * 3.6
      if cs.stockFcw and not prev:
        start = (t - t0, v, lat)
      if prev and not cs.stockFcw:
        eps.append((*start, t - t0 - start[0]))
      prev = cs.stockFcw
  print(f"{prefix} ({len(segs)} segs): {len(eps)} stockFcw episodes")
  for st, sp, la, d in eps:
    print(f"  t+{st:7.1f}s  {sp:5.1f} km/h  latActive={la}  lasted {d:.2f}s")
