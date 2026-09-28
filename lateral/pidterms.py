"""Torque controller terms during gentle curves, by speed, split by whether the car tracks or runs wide."""
import os, sys, warnings
from collections import defaultdict, deque
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
SPEEDS = [(10, 18), (18, 30), (30, 40), (40, 60), (60, 90)]
cells = defaultdict(list)
for prefix in sys.argv[1:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  st = {"v": 0.0, "lat": False}; last_press = -99; t0 = None; des_hist = deque(maxlen=40)
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
      if w == "carState":
        st["v"] = m.carState.vEgo * 3.6
        if m.carState.steeringPressed: last_press = t
      elif w == "carControl":
        st["lat"] = m.carControl.latActive
      elif w == "controlsState":
        cs = m.controlsState; ls = cs.lateralControlState
        des_hist.append(cs.desiredCurvature)
        if ls.which() != "torqueState" or not st["lat"] or t - last_press < 2 or len(des_hist) < 34:
          continue
        ts = ls.torqueState; des = des_hist[-34]; act = cs.curvature
        if not (0.002 <= abs(des) < 0.01) or abs(ts.output) > 0.6:
          continue
        sb = next((b for b in SPEEDS if b[0] <= st["v"] < b[1]), None)
        if not sb: continue
        sgn = 1.0 if des > 0 else -1.0
        r = act / des
        tag = "wide" if r < 0.75 else "ok"
        # sign the terms toward the requested turn so + means "helping the turn"
        cells[(sb, tag)].append((r, ts.error * sgn, ts.p * -sgn, ts.i * -sgn, ts.d * -sgn, ts.f * -sgn, ts.output * -sgn))
def med(xs):
  s = sorted(xs); return s[len(s) // 2] if s else float("nan")
print("gentle curves (radius 100-500 m), lateral active, no driver torque, not at torque cap")
print("terms signed so + pushes into the requested turn; units: lateral accel (m/s^2) except output")
print("  speed  track   n     ratio  error     P       I       D       FF      output")
for sb in SPEEDS:
  for tag in ("ok", "wide"):
    c = cells.get((sb, tag), [])
    if len(c) < 80: continue
    cols = list(zip(*c))
    print(f"  {sb[0]:>2}-{sb[1]:<2} {tag:5} {len(c):5d}  {med(cols[0]):5.2f}  {med(cols[1]):+.3f}  {med(cols[2]):+.3f}  {med(cols[3]):+.3f}  {med(cols[4]):+.3f}  {med(cols[5]):+.3f}  {med(cols[6]):+.3f}")
