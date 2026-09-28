"""Gentle curves under 40 km/h split by turn direction: tracking, feedforward and roll compensation.

Samples: openpilot steering, no driver torque for 2 s, 10-40 km/h, requested curvature 0.002-0.01 1/m
(radius 100-500 m), controller output under 0.6. Every term is signed so + points into the turn; the
second table shows how tracking changes as roll compensation toward the turn grows.

Usage: python controller_terms_by_turn_direction.py <route_id> [<route_id> ...]
"""
import math, os, sys, warnings
from collections import defaultdict, deque
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
cells = defaultdict(list)
offs = []
aoffs = []
for prefix in sys.argv[1:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  st = {"v": 0.0, "lat": False, "roll": 0.0, "aoff": 0.0}; last_press = -99; t0 = None; des_hist = deque(maxlen=40)
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
      if w == "carState":
        st["v"] = m.carState.vEgo * 3.6
        if m.carState.steeringPressed: last_press = t
      elif w == "carControl":
        st["lat"] = m.carControl.latActive
      elif w == "vehicleParameters":
        st["roll"] = m.vehicleParameters.roll; st["aoff"] = m.vehicleParameters.angleOffsetDeg; aoffs.append(st["aoff"])
      elif w == "lateralTorqueParameters":
        offs.append(m.lateralTorqueParameters.latAccelOffsetFiltered)
      elif w == "controlsState":
        cs = m.controlsState; ls = cs.lateralControlState
        des_hist.append(cs.desiredCurvature)
        # controlsState runs at 100 Hz: 34 samples is the ~0.34 s the achieved curvature trails the request
        if ls.which() != "torqueState" or not st["lat"] or t - last_press < 2 or len(des_hist) < 34:
          continue
        ts = ls.torqueState; des = des_hist[-34]; act = cs.curvature
        if not (0.002 <= abs(des) < 0.01) or abs(ts.output) > 0.6 or not (10 <= st["v"] < 40):
          continue
        side = "left " if des > 0 else "right"
        sgn = 1.0 if des > 0 else -1.0
        des_la = cs.desiredCurvature * (st["v"] / 3.6) ** 2
        roll_comp = st["roll"] * 9.81
        cells[side].append((act / des, ts.f * sgn, des_la * sgn, roll_comp * sgn, ts.p * sgn, ts.error * sgn, (ts.f - des_la + roll_comp) * sgn))
def med(xs):
  s = sorted(xs); return s[len(s) // 2] if s else float("nan")
print("gentle curves 10-40 km/h, lateral active, no driver torque; + = toward the requested turn (m/s^2)")
print("  turn    n     ratio  wide%   desiredLatAcc  rollComp  FF total   P       error   friction(FF-des+roll)")
for side, c in cells.items():
  cols = list(zip(*c))
  wide = 100 * sum(1 for r in cols[0] if r < 0.75) / len(c)
  print(f"  {side}  {len(c):5d}   {med(cols[0]):5.2f}  {wide:5.1f}   {med(cols[2]):+.3f}          {med(cols[3]):+.3f}   {med(cols[1]):+.3f}   {med(cols[4]):+.3f}  {med(cols[5]):+.3f}   {med(cols[6]):+.3f}")
if offs:
  print(f"latAccelOffsetFiltered median {med(offs):+.3f} m/s^2")
if aoffs:
  a = sorted(aoffs)
  print(f"angleOffsetDeg median {a[len(a)//2]:+.2f}  p10 {a[int(.1*len(a))]:+.2f}  p90 {a[int(.9*len(a))]:+.2f}")
print("\ntracking vs roll compensation toward the turn (all gentle curves 10-40 km/h, both sides):")
allc = [x for c in cells.values() for x in c]
for lo, hi in ((-9, 0.0), (0.0, 0.1), (0.1, 0.2), (0.2, 0.35), (0.35, 9)):
  b = [x for x in allc if lo <= x[3] < hi]
  if len(b) < 50: continue
  rs = sorted(x[0] for x in b)
  print(f"  rollComp {lo:+.2f}..{hi:+.2f}: n {len(b):5d}  ratio median {rs[len(rs)//2]:.2f}  wide {100 * sum(1 for r in rs if r < 0.75) / len(rs):4.1f}%")
