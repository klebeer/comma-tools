"""Driver torque of resting touches versus real maneuvers, to place the steering-pressed threshold.

Every steeringPressed episode with openpilot steering at 10 km/h or more is classified:
  maneuver  a blinker within 2 s of it, the wheel turned more than 20 deg during it, or lateral
            dropped within 2 s after it
  resting   everything else: the driver touched the wheel and openpilot kept the path
For each class it prints peak driver torque percentiles, then for candidate thresholds how many
resting episodes and how many maneuvers would still count as pressed (peak above the threshold).
The port's threshold today is STEER_THRESHOLD = 15 with a 5-frame debounce.

Usage: python steering_touch_profile.py <route_id> [<route_id> ...]
"""
import os, sys, warnings
from collections import deque
warnings.filterwarnings("ignore")
import numpy as np
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
MIN_KPH = 10.0
BLINKER_S, DROP_S, TURN_DEG = 2.0, 2.0, 20.0
THRESHOLDS = (15, 20, 25, 30, 35, 40, 50, 60)


def episodes(prefix):
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  out, cur = [], None
  st = {"lat": False, "v": 0.0}
  blinker_t = deque(maxlen=400); lat_off_t = []
  for s in segs:
    p = os.path.join(BASE, s, "rlog.zst")
    if not os.path.exists(p):
      continue
    for m in LogReader(p):
      w = m.which(); t = m.logMonoTime / 1e9
      if w == "carControl":
        if st["lat"] and not m.carControl.latActive:
          lat_off_t.append(t)
        st["lat"] = m.carControl.latActive
      elif w == "carState":
        cs = m.carState; st["v"] = cs.vEgo * 3.6
        if cs.leftBlinker or cs.rightBlinker:
          blinker_t.append(t)
        tq, ang = abs(cs.steeringTorque), cs.steeringAngleDeg
        if cs.steeringPressed and st["lat"] and st["v"] >= MIN_KPH:
          if cur is None:
            cur = {"t0": t, "peak": tq, "a0": ang, "amax": 0.0, "blink": any(t - b < BLINKER_S for b in blinker_t)}
          cur["peak"] = max(cur["peak"], tq); cur["amax"] = max(cur["amax"], abs(ang - cur["a0"])); cur["t1"] = t
        elif cur is not None:
          cur["blink"] = cur["blink"] or cs.leftBlinker or cs.rightBlinker
          out.append(cur); cur = None
  for e in out:
    e["dur"] = e["t1"] - e["t0"]
    dropped = any(e["t1"] <= x <= e["t1"] + DROP_S for x in lat_off_t)
    e["kind"] = "maneuver" if e["blink"] or e["amax"] > TURN_DEG or dropped else "resting"
  return out


def main():
  eps = []
  for prefix in sys.argv[1:]:
    ep = episodes(prefix)
    eps += ep
    print(f"{prefix}: {len(ep)} episodes, resting {sum(e['kind'] == 'resting' for e in ep)}")
  for kind in ("resting", "maneuver"):
    sub = [e for e in eps if e["kind"] == kind]
    if not sub:
      continue
    pk = np.array([e["peak"] for e in sub]); du = np.array([e["dur"] for e in sub])
    print(f"\n{kind}: n {len(sub)}  peak torque p25/p50/p75/p90 {np.percentile(pk, [25, 50, 75, 90]).round(0)}"
          f"  duration p50/p90 {np.percentile(du, [50, 90]).round(2)} s")
  res = [e["peak"] for e in eps if e["kind"] == "resting"]; man = [e["peak"] for e in eps if e["kind"] == "maneuver"]
  print("\nthreshold  resting still pressed  maneuvers still pressed")
  for th in THRESHOLDS:
    r = sum(p > th for p in res); m_ = sum(p > th for p in man)
    print(f"  {th:3d}      {r:4d}/{len(res):4d} ({100 * r / max(len(res), 1):3.0f}%)      "
          f"{m_:4d}/{len(man):4d} ({100 * m_ / max(len(man), 1):3.0f}%)")


main()
