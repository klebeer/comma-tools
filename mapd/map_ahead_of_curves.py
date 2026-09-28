"""What mapd reported ahead of specific low-speed tight curves (live log or replay).

The curves are hardcoded in EVENTS as (route_id, t+ seconds, replay file or None for the live
mapd log). For 10, 6, 3 and 0 s before each one it prints the tightest map curvature point ahead
within 150 m and the lowest map target speed.

Usage: python map_ahead_of_curves.py
"""
import bisect, glob, json, math, os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"

def hav(a, b):
  la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
  h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
  return 2 * 6371000 * math.asin(math.sqrt(h))

def brg(a, b):
  la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
  y = math.sin(lo2 - lo1) * math.cos(la2); x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(lo2 - lo1)
  return math.degrees(math.atan2(y, x)) % 360

def route_track(prefix, t_lo, t_hi):
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  t0 = None; out = []; v = 0.0; ang = 0.0
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
      if t < t_lo: continue
      if t > t_hi: return t0, out
      if w == "carState": v = m.carState.vEgo * 3.6; ang = m.carState.steeringAngleDeg
      elif w == "liveLocationKalman" and m.liveLocationKalman.positionGeodetic.valid:
        l = m.liveLocationKalman
        out.append((t, m.logMonoTime, l.positionGeodetic.value[0], l.positionGeodetic.value[1], math.degrees(l.calibratedOrientationNED.value[2]) % 360, v, ang))
  return t0, out

live = []
for f in glob.glob("/data/media/0/mapd_log/*.jsonl"):
  for l in open(f):
    try: live.append(json.loads(l))
    except ValueError: pass
live.sort(key=lambda r: r["mono_ns"]); live_t = [r["mono_ns"] for r in live]

def mapd_at(prefix, mono, rel_t, replay):
  if replay:
    i = bisect.bisect_right([r["t"] for r in replay], rel_t) - 1
    return replay[i] if i >= 0 else None
  i = bisect.bisect_right(live_t, mono) - 1
  # a live record older than 3 s belongs to another position
  return live[i] if i >= 0 and mono - live_t[i] < 3e9 else None

EVENTS = [("9de3eaeb9b", 318.0, None), ("9de3eaeb9b", 398.5, None), ("eb2f9b1e95", 88.5, "/data/mapd_eb2f9b1e95.jsonl"),
          ("938f815a7a", 227.0, "/data/mapd_1d.jsonl"), ("c540fbef83", 900.0, "/data/mapd_c540fbef83.jsonl")]
for prefix, te, rp in EVENTS:
  replay = [json.loads(l) for l in open(rp)] if rp else None
  t0, track = route_track(prefix, te - 12, te + 1)
  at_evt = min(track, key=lambda x: abs(x[0] - te))
  print(f"\n{prefix} t+{te}: {at_evt[5]:.0f} km/h, wheel {at_evt[6]:.0f} deg, pos {at_evt[2]:.5f},{at_evt[3]:.5f}  ({'replay' if rp else 'live mapd'})")
  for dt in (10, 6, 3, 0):
    p = min(track, key=lambda x: abs(x[0] - (te - dt)))
    r = mapd_at(prefix, p[1], p[0], replay)
    if not r:
      print(f"   -{dt:2d}s  no mapd record"); continue
    pos = (p[2], p[3]); best = None
    for q in r.get("MapCurvatures") or []:
      qq = (q["latitude"], q["longitude"]); d = hav(pos, qq)
      # ahead means within 90 degrees of the car's heading
      if d < 150 and abs((brg(pos, qq) - p[4] + 180) % 360 - 180) < 90 and (best is None or abs(q["curvature"]) > abs(best[0])):
        best = (q["curvature"], d)
    tv = [q["velocity"] * 3.6 for q in (r.get("MapTargetVelocities") or []) if hav(pos, (q["latitude"], q["longitude"])) < 150]
    road = r.get("RoadName")
    if best:
      print(f"   -{dt:2d}s  {p[5]:4.0f} km/h  max curvature ahead {abs(best[0]):.4f} (radius {1 / max(abs(best[0]), 1e-4):5.0f} m) at {best[1]:4.0f} m, "
            f"map target speed min {min(tv) if tv else float('nan'):4.0f} km/h, road {road}")
    else:
      print(f"   -{dt:2d}s  {p[5]:4.0f} km/h  no curvature point ahead within 150 m, road {road}")
