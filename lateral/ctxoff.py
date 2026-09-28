"""How often the right line is missing with a confident right edge, and how far right the car could sit."""
import os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
MARGIN = 0.8
def q(xs, p):
  s = sorted(xs); return s[min(len(s) - 1, int(p * len(s)))] if s else float("nan")
cond, ctrl, lat_n = [], [], 0
per_route = {}
for prefix in sys.argv[1:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  st = {"v": 0.0, "lat": False}; last_press = -99; t0 = None; n_c = 0; n_l = 0
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
      if w == "carState":
        st["v"] = m.carState.vEgo * 3.6
        if m.carState.steeringPressed: last_press = t
      elif w == "carControl":
        st["lat"] = m.carControl.latActive
      elif w == "modelV2" and st["lat"] and 15 < st["v"] < 50 and t - last_press > 2:
        md = m.modelV2
        if len(md.roadEdges) < 2 or len(md.laneLines) < 4: continue
        lat_n += 1; n_l += 1
        ll, rl, re = md.laneLines[1].y[0], md.laneLines[2].y[0], md.roadEdges[1].y[0]
        pl, pr, res = md.laneLineProbs[1], md.laneLineProbs[2], md.roadEdgeStds[1]
        if res > 1.0 or pl < 0.4: continue
        target = (ll + (re - MARGIN)) / 2          # centre between left line and curb minus margin
        rec = (target, re, re - rl, st["v"])
        if pr < 0.3:
          cond.append(rec); n_c += 1
        elif pr > 0.6:
          ctrl.append(rec)
  per_route[prefix] = (n_c, n_l)
print(f"lateral samples 15-50 km/h without driver torque: {lat_n}")
print(f"situation (right line < 0.3, left line > 0.4, confident right edge): {len(cond)} = {100 * len(cond) / max(lat_n, 1):.1f}% of them")
print("per route (situation / lateral samples):", {k: f"{a}/{b}" for k, (a, b) in per_route.items()})
for name, rows in (("SITUATION  (right line missing)", cond), ("CONTROL    (right line visible)", ctrl)):
  if not rows: continue
  sh = [r[0] for r in rows]; ed = [r[1] for r in rows]; gap = [r[2] for r in rows]
  print(f"\n{name}: n={len(rows)}")
  print(f"  camera -> right edge: median {q(ed, .5):.2f} m, p10 {q(ed, .1):.2f}, p90 {q(ed, .9):.2f}")
  print(f"  space between right line and edge: median {q(gap, .5):.2f} m, p90 {q(gap, .9):.2f}")
  print(f"  shift right to centre between left line and curb-{MARGIN}: median {q(sh, .5):+.2f} m, p25 {q(sh, .25):+.2f}, p75 {q(sh, .75):+.2f}, p90 {q(sh, .9):+.2f}")
  print(f"  share needing > 0.3 m: {100 * sum(1 for x in sh if x > 0.3) / len(sh):.0f}%   > 0.6 m: {100 * sum(1 for x in sh if x > 0.6) / len(sh):.0f}%   negative (already too far right): {100 * sum(1 for x in sh if x < 0) / len(sh):.0f}%")
