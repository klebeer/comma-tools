"""Clean evaluation of mapd curvature as a curve warning, across several replayed routes.

Usage on device:
  PYTHONPATH=/data/openpilot:/data/openpilot/openpilot /usr/local/venv/bin/python /data/mapd_eval2.py <route>:<replay.jsonl> ...
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
HORIZON_S = 6.0
MIN_AHEAD_M = 30.0
MIN_KMH = 50.0
MAX_KAPPA = 1 / 30.0          # tighter than a 30 m radius is junction geometry, not the road
LEAD_WINDOW_S = 12.0
DIRTY = {"laneChange", "preLaneChangeLeft", "preLaneChangeRight", "laneChangeBlocked", "steerOverride"}


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


def load_route(prefix):
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  t0 = None
  lat_acc, events = [], []
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which()
      t0 = m.logMonoTime if t0 is None else t0
      rel = (m.logMonoTime - t0) / 1e9
      if w == "liveLocationKalman":
        loc = m.liveLocationKalman
        vc, av = list(loc.velocityCalibrated.value), list(loc.angularVelocityCalibrated.value)
        if len(vc) >= 2 and len(av) == 3:
          lat_acc.append((rel, abs(math.hypot(vc[0], vc[1]) * av[2])))
      elif w == "onroadEvents":
        for e in m.onroadEvents:
          events.append((rel, str(e.name)))
  return lat_acc, events


def predict(r):
  v = r["v_kmh"] / 3.6
  pos, brg = (r["lat"], r["lon"]), r["bearing"] % 360
  horizon = max(MIN_AHEAD_M, v * HORIZON_S)
  kappa = 0.0
  for p in r.get("MapCurvatures") or []:
    q = (p["latitude"], p["longitude"])
    c = abs(p["curvature"])
    if c > MAX_KAPPA:
      continue
    if haversine(pos, q) <= horizon and abs((bearing_to(pos, q) - brg + 180) % 360 - 180) < 90:
      kappa = max(kappa, c)
  return v * v * kappa


def main():
  clean, sat_leads = [], []
  for arg in sys.argv[1:]:
    prefix, replay = arg.split(":")
    lat_acc, events = load_route(prefix)
    la_t = [t for t, _ in lat_acc]
    reps = [json.loads(line) for line in open(replay)]
    for r in reps:
      r["pred"] = predict(r)
    for r in reps:
      t = r["t"]
      if r["v_kmh"] < MIN_KMH:
        continue
      names = {n for te, n in events if t <= te <= t + HORIZON_S}
      if names & DIRTY:
        continue
      i0, i1 = bisect.bisect_left(la_t, t), bisect.bisect_right(la_t, t + HORIZON_S)
      actual = max((a for _, a in lat_acc[i0:i1]), default=0.0)
      clean.append((prefix, t, r["v_kmh"], r["pred"], actual))
    # every steerSaturated onset in the route, not only the sampled windows
    onsets, last = [], -99.0
    for te, n in events:
      if n == "steerSaturated":
        if te - last > 5:
          onsets.append(te)
        last = te
    for te in onsets:
      before = [r for r in reps if te - LEAD_WINDOW_S <= r["t"] <= te]
      v = next((r["v_kmh"] for r in reversed(before)), float("nan"))
      warned = [r for r in before if r["pred"] > 1.09 and r["v_kmh"] >= 30]
      lead = te - warned[0]["t"] if warned else None
      sat_leads.append((prefix, te, v, max((r["pred"] for r in before), default=0.0), lead))
    print(f"{prefix}: {len(reps)} positions, {len(onsets)} steerSaturated onsets")

  print(f"\nclean windows (>= {MIN_KMH:.0f} km/h, no lane change, no driver override): {len(clean)}")
  n_act = {th: sum(1 for c in clean if c[4] > th) for th in (1.09,)}
  print(f"  car actually above 1.09 m/s^2 in {n_act[1.09]} of them")
  pairs = [(c[3], c[4]) for c in clean]
  if len(pairs) > 2:
    mx = sum(p for p, _ in pairs) / len(pairs); my = sum(a for _, a in pairs) / len(pairs)
    sx = math.sqrt(sum((p - mx) ** 2 for p, _ in pairs)); sy = math.sqrt(sum((a - my) ** 2 for _, a in pairs))
    cov = sum((p - mx) * (a - my) for p, a in pairs)
    print(f"  correlation predicted vs actual: {cov / (sx * sy) if sx and sy else float('nan'):.2f}")
  print("  threshold on predicted | warnings | real (actual > 1.09) | precision | recall")
  for th in (0.6, 0.8, 1.0, 1.09, 1.3, 1.6):
    warn = [c for c in clean if c[3] > th]
    hit = [c for c in warn if c[4] > 1.09]
    rec = len(hit) / n_act[1.09] if n_act[1.09] else float("nan")
    prec = len(hit) / len(warn) if warn else float("nan")
    print(f"  {th:>21.2f} | {len(warn):8d} | {len(hit):20d} | {prec:9.2f} | {rec:6.2f}")
  print("\nsteerSaturated onsets: route, t, km/h, max predicted in the 12 s before, lead time of first warning >1.09")
  for p, te, v, mp, lead in sat_leads:
    print(f"  {p} t+{te:7.1f}s {v:5.0f} km/h  max_pred {mp:5.2f}  lead {'-' if lead is None else f'{lead:4.1f}s'}")


if __name__ == "__main__":
  main()
