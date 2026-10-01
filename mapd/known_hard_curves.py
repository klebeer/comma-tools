"""Hard curves learned from the driver's own routes, and how well they predict the next drive.

A hard moment is openpilot steering at 10 km/h or more, asking for more than 1/150 m of curvature,
and either sitting at the speed-dependent EPS ceiling, reaching under 75% of the request, or the
driver taking over. Moments within GROUP_S of each other count once, and each keeps its GPS fix.

Moments from every route are clustered by position (CLUSTER_M). A cluster seen on at least
MIN_ROUTES separate drives becomes a known hard curve, so one-off events do not qualify.

Coverage is scored leave-one-out: for each route, the curve list is rebuilt from the other routes
only, then each of that route's hard moments counts as covered when a known curve lies within
WARN_M ahead of it. That is what a warning built on this list would have achieved on a drive it
had never seen. The map-based rule is scored on the same moments from the shadow logs.

With --write <path> it also writes the qualifying curves as JSON, which curve_shadow reads.

Usage: python known_hard_curves.py [--write <path>] <route_id> [<route_id> ...]
"""
import calendar, glob, json, math, os, sys, time, warnings
from collections import defaultdict
warnings.filterwarnings("ignore")
import numpy as np
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
SHADOW = "/data/media/0/curve_shadow/*.jsonl"
CEIL = ([8.0, 8.5, 9.4, 10.3, 11.2, 12.1, 13.0, 13.9, 14.5], [1148, 1132, 1092, 1048, 1012, 920, 808, 676, 620])
MIN_KPH, CURVE_K, GROUP_S = 10.0, 1 / 150.0, 10.0
CLUSTER_M, MIN_ROUTES, WARN_M, LEAD_S = 25.0, 2, 120.0, 15.0


def hav(a, b):
  la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
  h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
  return 2 * 6371000 * math.asin(math.sqrt(h))


def scan(prefix):
  """(hard moments with a fix, shadow window, route span) for one route."""
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  st = {"v": 0.0, "press": False, "lat": False, "cmd": 0.0, "des": 0.0, "fix": None}
  was_pressed = False
  first = last = wall0 = None
  raw = []
  for s in segs:
    p = os.path.join(BASE, s, "rlog.zst")
    if not os.path.exists(p):
      continue
    for m in LogReader(p):
      w = m.which(); t = m.logMonoTime
      first = first or t; last = t
      if w == "initData" and wall0 is None:
        wall0 = m.initData.wallTimeNanos / 1e9
      elif w == "carState":
        cs = m.carState; st["v"] = cs.vEgo; st["press"] = cs.steeringPressed
        onset = st["press"] and not was_pressed; was_pressed = st["press"]
        if onset and st["lat"] and st["v"] * 3.6 >= MIN_KPH and st["des"] > CURVE_K and st["fix"]:
          raw.append((t, st["fix"], st["v"] * 3.6))
      elif w == "carControl":
        st["lat"] = m.carControl.latActive
      elif w == "carOutput":
        st["cmd"] = m.carOutput.actuatorsOutput.torqueOutputCan
      elif w == "liveLocationKalman":
        loc = m.liveLocationKalman
        if loc.positionGeodetic.valid:
          st["fix"] = (loc.positionGeodetic.value[0], loc.positionGeodetic.value[1])
      elif w == "controlsState" and st["lat"] and not st["press"]:
        cs = m.controlsState; kph = st["v"] * 3.6
        des, act = abs(cs.desiredCurvature), abs(cs.curvature)
        st["des"] = des
        if kph < MIN_KPH or des <= CURVE_K or not st["fix"]:
          continue
        at_ceiling = abs(st["cmd"]) >= 0.98 * float(np.interp(st["v"], *CEIL))
        if at_ceiling or act < 0.75 * des:
          raw.append((t, st["fix"], kph))
  moments, lt = [], -1e18
  for t, fix, kph in sorted(raw):
    if t - lt > GROUP_S * 1e9:
      moments.append((t, fix, kph))
    lt = t
  return moments, (first, last, wall0)


def cluster(points):
  """Greedy clustering of (fix, route) by CLUSTER_M; returns [(centre, {routes})]."""
  out = []
  for fix, route in points:
    for c in out:
      if hav(fix, c["centre"]) < CLUSTER_M:
        c["fixes"].append(fix); c["routes"].add(route)
        c["centre"] = (float(np.mean([f[0] for f in c["fixes"]])), float(np.mean([f[1] for f in c["fixes"]])))
        break
    else:
      out.append({"centre": fix, "fixes": [fix], "routes": {route}})
  return out


def shadow_warnings(span):
  first, last, wall0 = span
  wall1 = wall0 + (last - first) / 1e9
  out = []
  for f in glob.glob(SHADOW):
    started = calendar.timegm(time.strptime(os.path.basename(f)[:20], "%Y-%m-%d--%H-%M-%S"))
    if not wall0 - 120 <= started <= wall1:
      continue
    for line in open(f):
      try:
        w = json.loads(line)
      except ValueError:
        continue
      if first <= w.get("mono_ns", 0) <= last:
        out.append(w)
  return out


def main():
  args = sys.argv[1:]
  out_path = None
  if "--write" in args:
    i = args.index("--write"); out_path = args[i + 1]; args = args[:i] + args[i + 2:]
  routes = {}
  for prefix in args:
    moments, span = scan(prefix)
    if span[0] is None or span[2] is None:
      continue
    routes[prefix] = (moments, span)
    print(f"{prefix}: {len(moments)} hard moments")

  all_points = [(fix, r) for r, (ms, _) in routes.items() for _, fix, _ in ms]
  full = cluster(all_points)
  known = [c for c in full if len(c["routes"]) >= MIN_ROUTES]
  print(f"\n{len(all_points)} moments -> {len(full)} places, {len(known)} seen on {MIN_ROUTES}+ drives")
  for c in sorted(known, key=lambda c: -len(c["routes"]))[:15]:
    print(f"  {c['centre'][0]:+.5f},{c['centre'][1]:+.5f}  {len(c['routes'])} drives, {len(c['fixes'])} moments")

  tot = defaultdict(int)
  print("\nleave-one-out coverage per route   known-curve   map rule")
  for prefix, (moments, span) in routes.items():
    others = [(fix, r) for r, (ms, _) in routes.items() if r != prefix for _, fix, _ in ms]
    learned = [c["centre"] for c in cluster(others) if len(c["routes"]) >= MIN_ROUTES]
    warns = shadow_warnings(span)
    hit_known = sum(any(hav(fix, c) < WARN_M for c in learned) for _, fix, _ in moments)
    hit_map = sum(any(t - LEAD_S * 1e9 <= w["mono_ns"] <= t for w in warns) for t, _, _ in moments)
    tot["moments"] += len(moments); tot["known"] += hit_known; tot["map"] += hit_map
    tot["warns"] += len(warns)
    print(f"  {prefix}  {len(moments):3d} moments      {hit_known:3d}          {hit_map:3d}")
  if out_path:
    with open(out_path, "w") as f:
      json.dump([{"latitude": round(c["centre"][0], 6), "longitude": round(c["centre"][1], 6),
                  "drives": len(c["routes"]), "moments": len(c["fixes"])} for c in known], f, indent=1)
    print(f"\nwrote {len(known)} curves to {out_path}")

  m = max(tot["moments"], 1)
  print(f"\ntotal {tot['moments']} moments: known-curve list {tot['known']} ({100 * tot['known'] / m:.0f}%), "
        f"map rule {tot['map']} ({100 * tot['map'] / m:.0f}%, {tot['warns']} warnings)")


main()
