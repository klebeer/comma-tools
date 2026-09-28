"""Compare mapd's curvature ahead with what the car actually drove, per replayed position.

Predicted lateral acceleration is v^2 times the tightest map curvature ahead; actual is the
largest |v * yaw rate| in the next HORIZON_S. The summary gives the correlation and how often
each exceeds EPS_LAT_ACC; the table lists positions above 20 km/h where either is notable.

Usage: python map_vs_driven_curvature.py <route_prefix> <replay.jsonl>
"""
import bisect
import json
import math
import os
import sys
import warnings

warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
HORIZON_S = 6.0      # look-ahead window, seconds of travel at the current speed
MIN_AHEAD_M = 30.0   # below this the map horizon is too short to warn
EPS_LAT_ACC = 1.09   # the tune's MAX_LAT_ACCEL for this car, m/s^2


def haversine(a, b):
  r = 6371000.0
  la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
  h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
  return 2 * r * math.asin(math.sqrt(h))


def bearing_to(a, b):
  la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
  y = math.sin(lo2 - lo1) * math.cos(la2)
  x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(lo2 - lo1)
  return math.degrees(math.atan2(y, x)) % 360


def main():
  prefix, replay = sys.argv[1], sys.argv[2]
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  t0 = None
  lat_acc, events = [], []   # (t, |v * yaw_rate|), (t, name)
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which()
      t0 = m.logMonoTime if t0 is None else t0
      rel = (m.logMonoTime - t0) / 1e9
      if w == "liveLocationKalman":
        loc = m.liveLocationKalman
        vc = list(loc.velocityCalibrated.value)
        v = math.hypot(vc[0], vc[1]) if len(vc) >= 2 else 0.0
        if len(loc.angularVelocityCalibrated.value) == 3:
          lat_acc.append((rel, abs(v * loc.angularVelocityCalibrated.value[2])))
      elif w == "onroadEvents":
        for e in m.onroadEvents:
          if str(e.name) in ("steerSaturated", "steerOverride"):
            events.append((rel, str(e.name)))
  la_t = [t for t, _ in lat_acc]

  rows = []
  for line in open(replay):
    r = json.loads(line)
    v = r["v_kmh"] / 3.6
    pos, brg = (r["lat"], r["lon"]), r["bearing"] % 360
    pts = r.get("MapCurvatures") or []
    horizon = max(MIN_AHEAD_M, v * HORIZON_S)
    ahead = []
    for p in pts:
      q = (p["latitude"], p["longitude"])
      d = haversine(pos, q)
      rel_brg = abs((bearing_to(pos, q) - brg + 180) % 360 - 180)
      if d <= horizon and rel_brg < 90:
        ahead.append(abs(p["curvature"]))
    nearest = min((haversine(pos, (p["latitude"], p["longitude"])) for p in pts), default=float("nan"))
    kappa = max(ahead, default=0.0)
    pred = v * v * kappa
    i0, i1 = bisect.bisect_left(la_t, r["t"]), bisect.bisect_right(la_t, r["t"] + HORIZON_S)
    actual = max((a for _, a in lat_acc[i0:i1]), default=0.0)
    ev = sorted({n for t, n in events if r["t"] <= t <= r["t"] + HORIZON_S})
    rows.append((r["t"], r["v_kmh"], len(pts), nearest, len(ahead), kappa, pred, actual, ev, r.get("RoadName")))

  moving = [x for x in rows if x[1] > 20]
  with_map = [x for x in moving if x[2] > 0]
  print(f"{prefix}: {len(rows)} positions, {len(moving)} moving >20 km/h, {len(with_map)} with map curvature points")
  near = sorted(x[3] for x in with_map)
  if near:
    print(f"  distance from car to nearest map point: median {near[len(near)//2]:.0f} m, p90 {near[int(0.9*len(near))]:.0f} m")
  pairs = [(x[6], x[7]) for x in with_map if x[4] > 0]
  if pairs:
    n = len(pairs); mx = sum(p for p, _ in pairs) / n; my = sum(a for _, a in pairs) / n
    cov = sum((p - mx) * (a - my) for p, a in pairs)
    sx = math.sqrt(sum((p - mx) ** 2 for p, _ in pairs)); sy = math.sqrt(sum((a - my) ** 2 for _, a in pairs))
    print(f"  predicted vs actual max lat accel over next {HORIZON_S:.0f}s: n={n}, correlation {cov / (sx * sy) if sx and sy else float('nan'):.2f}")
  hi_pred = [x for x in with_map if x[6] > EPS_LAT_ACC]
  hi_act = [x for x in moving if x[7] > EPS_LAT_ACC]
  both = [x for x in hi_pred if x[7] > EPS_LAT_ACC]
  print(f"  map predicted > {EPS_LAT_ACC} m/s^2: {len(hi_pred)}; car actually > {EPS_LAT_ACC}: {len(hi_act)}; both: {len(both)}")
  sat = [x for x in moving if "steerSaturated" in x[8]]
  print(f"  windows with steerSaturated ahead: {len(sat)}, of which map predicted > {EPS_LAT_ACC}: {sum(1 for x in sat if x[6] > EPS_LAT_ACC)}")
  print("\n  t     km/h  pts nearest_m ahead  kappa   pred  actual  events  road")
  for x in rows:
    if x[1] > 20 and (x[6] > 0.5 or x[7] > 0.8 or x[8]):
      print(f"  {x[0]:6.0f} {x[1]:5.0f} {x[2]:4d} {x[3]:8.0f} {x[4]:5d} {x[5]:.4f} {x[6]:6.2f} {x[7]:6.2f}  {','.join(x[8]) or '-':24} {x[9]}")


if __name__ == "__main__":
  main()
