"""Where the car sits between the road edges on wide streets, with openpilot steering."""
import os, sys, warnings
from collections import defaultdict
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
cells = defaultdict(list); examples = []
for prefix in sys.argv[1:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  st = {"v": 0.0, "lat": False}; last_press = -99; t0 = None
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
      if w == "carState":
        st["v"] = m.carState.vEgo * 3.6
        if m.carState.steeringPressed: last_press = t
      elif w == "carControl":
        st["lat"] = m.carControl.latActive
      elif w == "modelV2" and st["lat"] and st["v"] > 15 and t - last_press > 2:
        md = m.modelV2
        if len(md.roadEdges) < 2 or len(md.laneLines) < 4: continue
        le, re = md.roadEdges[0].y[0], md.roadEdges[1].y[0]
        les, res = md.roadEdgeStds[0], md.roadEdgeStds[1]
        if les > 0.6 or res > 0.6: continue
        width = re - le
        if width < 3: continue
        frac_from_right = re / width           # 0 = at the right edge, 0.5 = road centre, 1 = left edge
        lanes_seen = min(md.laneLineProbs[1], md.laneLineProbs[2])
        band = "< 6 m" if width < 6 else "6-8 m" if width < 8 else "8-11 m" if width < 11 else ">= 11 m"
        cells[band].append((frac_from_right, re, width, lanes_seen))
        if width >= 8 and frac_from_right > 0.45 and len(examples) < 12 and (not examples or t - examples[-1][1] > 5):
          examples.append((prefix, round(t, 1), round(st["v"]), round(width, 1), round(re, 2), round(frac_from_right, 2), round(lanes_seen, 2)))
def q(xs, p):
  s = sorted(xs); return s[min(len(s) - 1, int(p * len(s)))]
print("openpilot steering, >15 km/h, no driver torque, both road edges confident")
print("  road width   n     camera->right edge (m)  position (0=right edge, 0.5=centre)  in left half  lines seen")
for band in ("< 6 m", "6-8 m", "8-11 m", ">= 11 m"):
  c = cells.get(band, [])
  if len(c) < 40: continue
  fr = [x[0] for x in c]; rd = [x[1] for x in c]
  print(f"  {band:8} {len(c):6d}   {q(rd, .5):5.2f} (p90 {q(rd, .9):5.2f})          {q(fr, .5):.2f} (p90 {q(fr, .9):.2f})                 {100 * sum(1 for f in fr if f > 0.5) / len(fr):5.1f}%     {q([x[3] for x in c], .5):.2f}")
print("wide-street moments past 45% toward the left:", examples)
