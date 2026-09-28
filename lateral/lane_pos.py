"""Where the car actually sits in its lane, per speed band, undoing CameraOffset and the context shift.

The camera offset shears the model input, so model outputs are in a virtual frame: a real lateral
position Y appears at Y - o, with o the applied offset (left positive). This adds o back, replaying
ContextOffset from the logged model outputs to know the shift at each frame.
"""
import os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
from openpilot.sunnypilot.modeld_v2.context_offset import ContextOffset
BASE = "/data/media/0/realdata"
base_offset = float(sys.argv[1])

def q(xs, p):
  s = sorted(xs); return s[min(len(s) - 1, int(p * len(s)))] if s else float("nan")

for prefix in sys.argv[2:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  ctx = ContextOffset("MAZDA_CX5_2022_NON_MRCC", 0.05)
  v, lat, last_press, t0, applied = 0.0, False, -99.0, None, 0.0
  bands = {}
  for s in segs:
    p = os.path.join(BASE, s, "rlog.zst")
    if not os.path.exists(p): continue
    for m in LogReader(p):
      w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
      if w == "carState":
        v = m.carState.vEgo
        if m.carState.steeringPressed: last_press = t
      elif w == "carControl":
        lat = m.carControl.latActive
      elif w == "modelV2":
        md = m.modelV2
        o = applied
        shift = ctx.update(md, v)
        applied = 0.9 * applied + 0.1 * ctx.camera_offset(base_offset)
        if not lat or t - last_press < 2 or len(md.laneLines) < 4: continue
        kph = v * 3.6
        pl, pr = md.laneLineProbs[1], md.laneLineProbs[2]
        yl, yr = md.laneLines[1].y[0] + o, md.laneLines[2].y[0] + o
        if kph < 15 or pl < 0.5: continue
        band = "15-50 " + ("both lines" if pr > 0.6 else "no right line" if pr < 0.3 else "weak right") if kph < 50 else \
               "50-70" if kph < 70 else ">70"
        b = bands.setdefault(band, {"c": [], "l": [], "w": [], "sh": []})
        if pr > 0.6:
          b["c"].append((yl + yr) / 2); b["w"].append(yr - yl)
        b["l"].append(-yl); b["sh"].append(shift)
  print(f"\n{prefix}  (car position: + = car left of lane centre; left gap = camera to left line)")
  for band in sorted(bands):
    b = bands[band]; n = len(b["l"])
    print(f"  {band:22s} n={n:5d}  pos {q(b['c'], .5):+.2f} m (p25 {q(b['c'], .25):+.2f}, p75 {q(b['c'], .75):+.2f})  "
          f"left gap {q(b['l'], .5):.2f} m  lane {q(b['w'], .5):.2f} m  ctx shift>0.05: {100 * sum(x > .05 for x in b['sh']) / max(n, 1):.0f}%"
          f" (shift p50 {q(b['sh'], .5):.2f}, p90 {q(b['sh'], .9):.2f})")
