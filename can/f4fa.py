"""Timeline of the 2-bit field in 0x4FA byte 4 (DBC bits 38:37) with driving context."""
import os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
for prefix in sys.argv[1:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  t0 = None
  ctx = {"v": 0.0, "lat": 0, "press": 0, "blk": "", "yl": float("nan"), "yr": float("nan"), "pl": 0.0, "pr": 0.0}
  cur = {}
  eps = []
  for s in segs:
    p = os.path.join(BASE, s, "rlog.zst")
    if not os.path.exists(p):
      continue
    for m in LogReader(p):
      w = m.which()
      t = m.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      rel = t - t0
      if w == "carState":
        cs = m.carState
        ctx["v"] = cs.vEgo * 3.6
        ctx["press"] = int(cs.steeringPressed)
        ctx["blk"] = ("L" if cs.leftBlinker else "") + ("R" if cs.rightBlinker else "")
      elif w == "carControl":
        ctx["lat"] = int(m.carControl.latActive)
      elif w == "modelV2":
        md = m.modelV2
        if len(md.laneLines) >= 4:
          ctx["yl"], ctx["yr"] = md.laneLines[1].y[0], md.laneLines[2].y[0]
          ctx["pl"], ctx["pr"] = md.laneLineProbs[1], md.laneLineProbs[2]
      elif w == "can":
        for c in m.can:
          if c.address != 0x4FA or c.src not in (0, 2):
            continue
          val = (c.dat[4] >> 5) & 3
          old = cur.get(c.src)
          if old is not None and val != old and c.src == 0:
            eps.append((rel, old, val, dict(ctx)))
          cur[c.src] = val
  print(f"{prefix} ({len(segs)} segs): {len(eps)} transitions of 0x4FA[38:37] on bus 0, last bus2 value {cur.get(2)}")
  start = None
  for rel, old, val, c in eps:
    if old == 0:
      start = rel
    dur = f"  lasted {rel - start:5.1f}s" if val == 0 and start is not None else ""
    print(f"  t+{rel:7.1f}s {old}->{val}  {c['v']:5.1f} km/h lat={c['lat']} press={c['press']} blk={c['blk'] or '-':2} "
          f"leftLine {c['yl']:+.2f}m (p{c['pl']:.2f}) rightLine {c['yr']:+.2f}m (p{c['pr']:.2f}){dur}")
