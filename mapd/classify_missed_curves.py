"""Why the shadow curve warning missed each hard moment, from what mapd had ahead at the time.

Hard moments are grouped torque-cap or running-wide episodes, as in evaluate_curve_warning.py.
Each one is classified by the first cause that applies:
  intersection  heading changes over 45 deg across the moment and the road name changes: a turn onto
                another street, which a curvature map of the current road cannot show
  gated         mapd showed a curve tighter than 45 m within 100 m ahead in the 15 s before, but the
                shadow was not armed (lateral off, or speed outside 10-60 km/h)
  radius 45-100 mapd's tightest point ahead was 45-100 m: the rule's radius threshold left it out
  no map curve  nothing under 100 m radius ahead in the mapd log
  no map data   no mapd record in the 15 s before
Warned moments are listed too, so the split covers every hard moment.

Usage: python classify_missed_curves.py <route_id> [<route_id> ...]
"""
import bisect, calendar, glob, json, math, os, sys, time, warnings
from collections import Counter
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
SHADOW = "/data/media/0/curve_shadow/*.jsonl"
MAPD = "/data/media/0/mapd_log/*.jsonl"
LEAD_S = 15.0
TIGHT_K = 1 / 60.0
RULE_RADIUS_M, WIDE_RADIUS_M, LOOKAHEAD_M = 45.0, 100.0, 100.0
MIN_KPH, MAX_KPH = 10.0, 60.0


def file_start(f):
  return calendar.timegm(time.strptime(os.path.basename(f)[:20], "%Y-%m-%d--%H-%M-%S"))


def hav(a, b):
  la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
  h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
  return 2 * 6371000 * math.asin(math.sqrt(h))


def brg(a, b):
  la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
  y = math.sin(lo2 - lo1) * math.cos(la2)
  x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(lo2 - lo1)
  return math.degrees(math.atan2(y, x)) % 360


def tightest_ahead(pos, heading, curvatures):
  best = None
  for p in curvatures or []:
    k = abs(p.get("curvature", 0.0))
    if k <= 0:
      continue
    q = (p["latitude"], p["longitude"])
    if hav(pos, q) > LOOKAHEAD_M:
      continue
    if abs((brg(pos, q) - heading + 180) % 360 - 180) > 90:
      continue
    best = 1 / k if best is None else min(best, 1 / k)
  return best


def load(pattern, wall0, wall1):
  out = []
  for f in glob.glob(pattern):
    if not wall0 - 120 <= file_start(f) <= wall1:
      continue
    for line in open(f):
      try:
        out.append(json.loads(line))
      except ValueError:
        pass
  return out


def main():
  totals = Counter()
  for prefix in sys.argv[1:]:
    segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
    if not segs:
      continue
    first = last = wall0 = None; hard = []; st = {"lat": False, "v": 0.0}; prev = 0.0
    track = []  # (mono_ns, lat, lon, heading, v_kph, lat_active)
    for s in segs:
      for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
        w = m.which(); t = m.logMonoTime
        first = first or t; last = t
        if w == "initData" and wall0 is None:
          wall0 = m.initData.wallTimeNanos / 1e9
        elif w == "carState":
          st["v"] = m.carState.vEgo * 3.6
        elif w == "carControl":
          st["lat"] = m.carControl.latActive
        elif w == "liveLocationKalman":
          loc = m.liveLocationKalman
          if loc.positionGeodetic.valid:
            track.append((t, loc.positionGeodetic.value[0], loc.positionGeodetic.value[1],
                          math.degrees(loc.calibratedOrientationNED.value[2]) % 360, st["v"], st["lat"]))
        elif w == "controlsState" and st["lat"]:
          cs = m.controlsState; ls = cs.lateralControlState
          o = getattr(ls, ls.which()).output if ls.which() == "torqueState" else 0.0
          if abs(o) > 0.6 and abs(abs(o) - abs(prev)) < 1e-4:
            hard.append(t)
          prev = o
          if abs(cs.desiredCurvature) > TIGHT_K and abs(cs.curvature) < 0.75 * abs(cs.desiredCurvature):
            hard.append(t)
    wall1 = wall0 + (last - first) / 1e9
    warns = [w for w in load(SHADOW, wall0, wall1) if first <= w["mono_ns"] <= last]
    mapd = sorted((r for r in load(MAPD, wall0, wall1) if first <= r.get("mono_ns", 0) <= last), key=lambda r: r["mono_ns"])
    mapd_t = [r["mono_ns"] for r in mapd]
    track_t = [x[0] for x in track]

    moments, last_t = [], -1e18
    for t in sorted(hard):
      if t - last_t > 10e9:
        moments.append(t)
      last_t = t

    def at(t):
      i = min(max(bisect.bisect_left(track_t, t), 0), len(track) - 1)
      return track[i] if track else None

    def road(t):
      i = bisect.bisect_right(mapd_t, t) - 1
      return mapd[i].get("RoadName") if i >= 0 else None

    print(f"\n=== {prefix}: {len(moments)} hard moments, {len(warns)} warnings, {len(mapd)} mapd records")
    for t in moments:
      rel = (t - first) / 1e9
      if any(t - LEAD_S * 1e9 <= w["mono_ns"] <= t for w in warns):
        cause, detail = "warned", ""
      else:
        a, b = at(t - 5e9), at(t + 5e9)
        turn = abs((b[3] - a[3] + 180) % 360 - 180) if a and b else 0
        name_change = road(t - 5e9) != road(t + 8e9)
        recs = [r for r in mapd[bisect.bisect_left(mapd_t, t - LEAD_S * 1e9):bisect.bisect_right(mapd_t, t)]]
        radii, armed = [], []
        for r in recs:
          p = at(r["mono_ns"])
          try:
            curv = json.loads(r["MapCurvatures"]) if isinstance(r.get("MapCurvatures"), str) else r.get("MapCurvatures")
          except ValueError:
            curv = None
          rad = tightest_ahead((p[1], p[2]), p[3], curv) if p else None
          if rad is not None:
            radii.append(rad); armed.append(p[5] and MIN_KPH <= p[4] <= MAX_KPH)
        tight = [a_ for rad, a_ in zip(radii, armed) if rad < RULE_RADIUS_M]
        if turn > 45 and name_change:
          cause = "intersection"
        elif tight and not any(tight):
          cause = "gated"
        elif tight:
          cause = "tight but no warning"
        elif radii and min(radii) < WIDE_RADIUS_M:
          cause = "radius 45-100"
        elif recs:
          cause = "no map curve"
        else:
          cause = "no map data"
        p = at(t)
        detail = f"{p[4]:4.0f} km/h turn {turn:4.0f} deg  min map radius {min(radii):5.1f} m" if radii and p else \
                 (f"{p[4]:4.0f} km/h turn {turn:4.0f} deg" if p else "")
        detail += f"  {road(t - 5e9)} -> {road(t + 8e9)}" if name_change else f"  {road(t)}"
      totals[cause] += 1
      print(f"  t+{rel:6.0f}s {cause:22} {detail}")
  print("\ntotal:", dict(totals))


main()
