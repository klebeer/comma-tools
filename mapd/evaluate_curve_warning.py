"""Judge curve_shadow's would-be warnings against what the car did next, and find missed curves.

A warning is real when, within OUTCOME_S, openpilot's torque output pinned at its cap, the driver
took over, or the car tracked under 75% of a requested curvature tighter than 1/60 m. A tight
moment with no warning in the LEAD_S before it is a miss.

Each warning prints as REAL with the outcomes seen, or "no outcome". The last line counts the
hard moments (torque cap or running wide, grouped within 10 s) and lists those with no warning.
Warnings come from the curve_shadow logs in /data/media/0/curve_shadow.

Usage: python evaluate_curve_warning.py <route_id> ...
"""
import glob, json, os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
SHADOW = "/data/media/0/curve_shadow/*.jsonl"
OUTCOME_S, LEAD_S = 20.0, 15.0
TIGHT_K = 1 / 60.0   # 60 m radius

warns = []
for f in glob.glob(SHADOW):
  for line in open(f):
    try:
      warns.append(json.loads(line))
    except ValueError:
      pass

for prefix in sys.argv[1:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  if not segs:
    continue
  first = last = None; hard = []; st = {"lat": False, "v": 0.0}; prev = 0.0
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which(); t = m.logMonoTime
      first = first or t; last = t
      if w == "carState":
        st["v"] = m.carState.vEgo * 3.6
        if m.carState.steeringPressed and st["lat"]:
          hard.append((t, "takeover"))
      elif w == "carControl":
        st["lat"] = m.carControl.latActive
      elif w == "controlsState" and st["lat"]:
        cs = m.controlsState; ls = cs.lateralControlState
        o = getattr(ls, ls.which()).output if ls.which() == "torqueState" else 0.0
        # an output held exactly constant above 0.6 means it is clipped at the torque cap
        if abs(o) > 0.6 and abs(abs(o) - abs(prev)) < 1e-4:
          hard.append((t, "torque cap"))
        prev = o
        if abs(cs.desiredCurvature) > TIGHT_K and abs(cs.curvature) < 0.75 * abs(cs.desiredCurvature):
          hard.append((t, "runs wide"))
  mine = [w for w in warns if first <= w["mono_ns"] <= last]
  print(f"{prefix}: {len(mine)} would-be warnings")
  for w in mine:
    after = sorted({k for t, k in hard if w["mono_ns"] <= t <= w["mono_ns"] + OUTCOME_S * 1e9})
    rel = (w["mono_ns"] - first) / 1e9
    print(f"  t+{rel:6.1f}s {w['v_kph']:4.0f} km/h radius {w['radius_m']:5.1f} m at {w['distance_m']:5.1f} m  "
          f"{'REAL ' + ','.join(after) if after else 'no outcome'}  {w.get('road')}")
  # tight moments with a torque cap or a wide run, grouped, and whether a warning preceded them
  moments, last_t = [], -1e18
  for t, k in sorted(hard):
    if k == "takeover":
      continue
    if t - last_t > 10e9:
      moments.append(t)
    last_t = t
  missed = [t for t in moments if not any(t - LEAD_S * 1e9 <= w["mono_ns"] <= t for w in mine)]
  print(f"  hard moments {len(moments)}, preceded by a warning {len(moments) - len(missed)}, missed at t+: "
        f"{[round((t - first) / 1e9) for t in missed]}")
