"""Lane centre offset and lane line confidence while openpilot steers above 40 km/h.

Share of model frames with both inner lines above 0.7 probability and with at least one below
0.3; for the confident frames, the lane centre offset and lane width; and the last
liveCalibration.

Usage: python lane_center_and_confidence.py <route_id> [<route_id> ...]
"""
import os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"

def pct(xs, q):
  s = sorted(xs)
  return s[min(len(s) - 1, int(q * len(s)))] if s else float("nan")

for prefix in sys.argv[1:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  v, lat, calib = 0.0, False, None
  offs, widths, probs, edge_probs, n_model = [], [], [], [], 0
  for s in segs:
    p = os.path.join(BASE, s, "rlog.zst")
    if not os.path.exists(p):
      continue
    for m in LogReader(p):
      w = m.which()
      if w == "carState":
        v = m.carState.vEgo * 3.6
      elif w == "carControl":
        lat = m.carControl.latActive
      elif w == "liveCalibration":
        c = m.liveCalibration
        calib = (str(c.calStatus), c.calPerc, [round(x, 4) for x in c.rpyCalib], c.validBlocks, list(c.height) if len(c.height) else None)
      elif w == "modelV2" and lat and v > 40:
        md = m.modelV2
        if len(md.laneLines) < 4:
          continue
        n_model += 1
        lp = list(md.laneLineProbs)
        probs.append((lp[1], lp[2]))
        edge_probs.append(max(md.roadEdgeStds[0], md.roadEdgeStds[1]) if len(md.roadEdgeStds) > 1 else float("nan"))
        if lp[1] > 0.7 and lp[2] > 0.7:
          yl, yr = md.laneLines[1].y[0], md.laneLines[2].y[0]
          offs.append((yl + yr) / 2)
          widths.append(yr - yl)
  good = sum(1 for a, b in probs if a > 0.7 and b > 0.7)
  weak = sum(1 for a, b in probs if a < 0.3 or b < 0.3)
  print(f"{prefix}: model frames lat active >40 km/h: {n_model}")
  if n_model:
    print(f"  both inner lines prob>0.7: {100 * good / n_model:.1f}%   at least one <0.3: {100 * weak / n_model:.1f}%")
  if offs:
    print(f"  lane center rel. to car (m, +right): median {pct(offs, 0.5):+.3f}  p10 {pct(offs, 0.1):+.3f}  p90 {pct(offs, 0.9):+.3f}  n={len(offs)}")
    print(f"  lane width (m): median {pct(widths, 0.5):.2f}")
  print(f"  liveCalibration: {calib}")
