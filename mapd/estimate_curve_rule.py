"""Replay the fixed-radius curve warning and a speed-aware one over the same mapd records.

Capability: for each speed band, the curvature the car actually reached while the commanded
torque sat at the speed-dependent EPS ceiling (lateral active, no driver torque, requesting more
than it got). The median of those samples is what the EPS can hold at that speed.

Rules, both armed only with lateral active at 10-60 km/h, looking 100 m ahead within 90 deg:
  fixed   map radius under 45 m (the rule running in shadow mode today)
  speed@m map curvature above m x the capability at the current speed, for each m in MARGINS
Holdoff: FIXED_HOLDOFF_S for the fixed rule (as deployed), SPEED_HOLDOFF_S for the new one, and
once per curve point within 30 m for 60 s.

Hard moments: torque at the ceiling, running wide (under 75% of the requested curvature) or a
driver takeover, at 10 km/h or more with a requested curvature above 1/150 m, grouped within 10 s.
A takeover counts because a driver who steps in before the ceiling is the outcome being predicted. A warning is real
when a hard moment follows within 20 s; a moment is caught when a warning came in the 15 s before.

Usage: python estimate_curve_rule.py <route_id> [<route_id> ...]
"""
import bisect, calendar, glob, json, math, os, sys, time, warnings
from collections import defaultdict
warnings.filterwarnings("ignore")
import numpy as np
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
MAPD = "/data/media/0/mapd_log/*.jsonl"
# opendbc EPS_CEILING_LOOKUP, m/s to CAN counts
CEIL = ([8.0, 8.5, 9.4, 10.3, 11.2, 12.1, 13.0, 13.9, 14.5], [1148, 1132, 1092, 1048, 1012, 920, 808, 676, 620])
BANDS = [(10, 20), (20, 30), (30, 40), (40, 50), (50, 60)]
LOOKAHEAD_M, MIN_KPH, MAX_KPH = 100.0, 10.0, 60.0
FIXED_RADIUS_M = 45.0
MARGINS = (0.7, 0.9, 1.1)
FIXED_HOLDOFF_S, SPEED_HOLDOFF_S, SAME_CURVE_S, SAME_CURVE_M = 20.0, 8.0, 60.0, 30.0
OUTCOME_S, LEAD_S, GROUP_S = 20.0, 15.0, 10.0
CURVE_K = 1 / 150.0


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


def band(kph):
  return next((b for b in BANDS if b[0] <= kph < b[1]), None)


def read_route(prefix):
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  first = last = wall0 = None; st = {"v": 0.0, "lat": False, "press": False, "cmd": 0.0, "des": 0.0}; was_pressed = False
  track, hard, cap = [], [], []
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
        st["v"] = m.carState.vEgo; st["press"] = m.carState.steeringPressed
        onset = st["press"] and not was_pressed; was_pressed = st["press"]
        if onset and st["lat"] and st["v"] * 3.6 >= MIN_KPH and st["des"] > CURVE_K:
          hard.append(t)
      elif w == "carControl":
        st["lat"] = m.carControl.latActive
      elif w == "carOutput":
        st["cmd"] = m.carOutput.actuatorsOutput.torqueOutputCan
      elif w == "liveLocationKalman":
        loc = m.liveLocationKalman
        if loc.positionGeodetic.valid:
          track.append((t, loc.positionGeodetic.value[0], loc.positionGeodetic.value[1],
                        math.degrees(loc.calibratedOrientationNED.value[2]) % 360, st["v"] * 3.6, st["lat"]))
      elif w == "controlsState" and st["lat"] and not st["press"]:
        cs = m.controlsState; kph = st["v"] * 3.6
        des, act = abs(cs.desiredCurvature), abs(cs.curvature)
        st["des"] = des
        at_ceiling = abs(st["cmd"]) >= 0.98 * float(np.interp(st["v"], *CEIL))
        if at_ceiling and des > act and kph >= MIN_KPH:
          cap.append((kph, act))
        if kph >= MIN_KPH and des > CURVE_K and (at_ceiling or act < 0.75 * des):
          hard.append(t)
  return first, last, wall0, track, hard, cap


def load_mapd(first, last, wall0):
  wall1 = wall0 + (last - first) / 1e9
  recs = []
  for f in glob.glob(MAPD):
    if not wall0 - 120 <= file_start(f) <= wall1:
      continue
    for line in open(f):
      try:
        r = json.loads(line)
      except ValueError:
        continue
      if first <= r.get("mono_ns", 0) <= last and r.get("MapCurvatures"):
        recs.append(r)
  return sorted(recs, key=lambda r: r["mono_ns"])


def replay(recs, track, capability, rule, margin=1.0):
  track_t = [x[0] for x in track]
  out, last_t, last_pt = [], -1e18, None
  holdoff = FIXED_HOLDOFF_S if rule == "fixed" else SPEED_HOLDOFF_S
  for r in recs:
    i = bisect.bisect_left(track_t, r["mono_ns"])
    if not track or i >= len(track):
      continue
    _, lat, lon, heading, kph, lat_active = track[i]
    if not lat_active or not MIN_KPH <= kph <= MAX_KPH:
      continue
    limit_k = 1 / FIXED_RADIUS_M if rule == "fixed" else margin * capability(kph)
    best = None
    for p in r["MapCurvatures"]:
      k = abs(p.get("curvature", 0.0)); q = (p["latitude"], p["longitude"])
      if k <= limit_k or hav((lat, lon), q) > LOOKAHEAD_M:
        continue
      if abs((brg((lat, lon), q) - heading + 180) % 360 - 180) > 90:
        continue
      if best is None or k > best[0]:
        best = (k, q)
    if best is None:
      continue
    t = r["mono_ns"]
    if t - last_t < holdoff * 1e9:
      continue
    if last_pt is not None and t - last_t < SAME_CURVE_S * 1e9 and hav(best[1], last_pt) < SAME_CURVE_M:
      continue
    out.append((t, kph, 1 / best[0])); last_t, last_pt = t, best[1]
  return out


def main():
  routes = {}
  cap_by_band = defaultdict(list)
  for prefix in sys.argv[1:]:
    first, last, wall0, track, hard, cap = read_route(prefix)
    if first is None:
      continue
    routes[prefix] = (first, last, wall0, track, hard)
    for kph, act in cap:
      b = band(kph)
      if b:
        cap_by_band[b].append(act)

  print("capability: curvature held with the torque at the EPS ceiling")
  table = {}
  for b in BANDS:
    xs = cap_by_band.get(b, [])
    if len(xs) >= 50:
      table[b] = float(np.median(xs))
      print(f"  {b[0]:>2}-{b[1]:<2} km/h  n {len(xs):5d}  median {table[b]:.4f} 1/m (radius {1 / table[b]:5.1f} m)"
            f"  p25 {np.percentile(xs, 25):.4f}  p75 {np.percentile(xs, 75):.4f}")
    else:
      print(f"  {b[0]:>2}-{b[1]:<2} km/h  n {len(xs):5d}  too few samples")
  known = sorted(table)
  mids = [(b[0] + b[1]) / 2 for b in known]; vals = [table[b] for b in known]

  def capability(kph):
    return float(np.interp(kph, mids, vals)) if vals else 1 / FIXED_RADIUS_M

  rules = [("fixed", 1.0)] + [(f"speed@{m}", m) for m in MARGINS]
  totals = {name: defaultdict(float) for name, _ in rules}
  for prefix, (first, last, wall0, track, hard) in routes.items():
    moments, lt = [], -1e18
    for t in sorted(hard):
      if t - lt > GROUP_S * 1e9:
        moments.append(t)
      lt = t
    recs = load_mapd(first, last, wall0)
    line = f"{prefix}: {len(moments)} hard moments, {len(recs)} mapd records"
    for rule, margin in rules:
      ws = replay(recs, track, capability, "fixed" if rule == "fixed" else "speed", margin)
      real = [w for w in ws if any(w[0] <= m <= w[0] + OUTCOME_S * 1e9 for m in moments)]
      caught = [m for m in moments if any(m - LEAD_S * 1e9 <= w[0] <= m for w in ws)]
      leads = [(m - max(w[0] for w in ws if m - LEAD_S * 1e9 <= w[0] <= m)) / 1e9 for m in caught]
      tt = totals[rule]
      tt["warnings"] += len(ws); tt["real"] += len(real); tt["moments"] += len(moments); tt["caught"] += len(caught)
      tt["lead_sum"] += sum(leads)
      line += f" | {rule}: {len(ws)} warn, {len(real)} real, caught {len(caught)}"
    print(line)

  print()
  for rule, tt in totals.items():
    prec = tt["real"] / tt["warnings"] if tt["warnings"] else float("nan")
    rec = tt["caught"] / tt["moments"] if tt["moments"] else float("nan")
    lead = tt["lead_sum"] / tt["caught"] if tt["caught"] else float("nan")
    print(f"{rule:9}  warnings {int(tt['warnings']):3d}  real {int(tt['real']):3d} ({100 * prec:4.0f}%)  "
          f"moments caught {int(tt['caught'])}/{int(tt['moments'])} ({100 * rec:4.0f}%)  mean lead {lead:4.1f} s")


main()
