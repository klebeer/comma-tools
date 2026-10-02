"""How many hard steering moments would a route rule have warned about, next to the map and learned rules?

The three rules of the curve warning are replayed over each drive the way curve_shadow applies
them: lateral active at 10-60 km/h, the nearest hit leads, one warning per HOLDOFF_S and one per
curve per SAME_CURVE_S.
  map      mapd sees a curve tighter than MIN_RADIUS_M within LOOKAHEAD_M (from the mapd logs)
  learned  a known hard curve lies ahead, the list rebuilt from the other drives only
  route    the route follower has a turn or a roundabout ahead within LOOKAHEAD_M, on a bundle
           planned for this drive from the other drives' ways plus the planner's routes
Each rule is scored alone, then map+learned (what runs in shadow today) and all three.

Hard moments are known_hard_curves': torque at the EPS ceiling, running wide or a takeover, at
10 km/h or more with a requested curvature above 1/150 m, grouped within 10 s. A moment is caught
when a warning pointed at a place within MATCH_M of it, at most LEAD_S before it. A warning that
caught nothing is false. Only drives with mapd logs are scored, so the three rules see the same
moments.

--holdoff S replays with another gap between warnings: 0 shows what the rules know, without the
limit of one warning every HOLDOFF_S that an audible alert needs.
--no-history plans with the planner's routes only. Read with map+route, that is a street never
driven before: no learned curves and no past routes, only the map and the destination.
--all-drives takes no destination: the bundle is every other drive, whole.

Usage: python mapd/route_hard_moments.py [--holdoff S] [--no-history | --all-drives] <route_id> [<route_id> ...]
"""
import bisect, calendar, glob, json, math, os, sys, time, warnings
from collections import defaultdict
warnings.filterwarnings("ignore")
import numpy as np
from openpilot.sunnypilot.mapd.curve_warning import LOOKAHEAD_M, known_curve_ahead, tight_curve_ahead
from openpilot.tools.lib.logreader import LogReader
from comma_nav import history as nav_history
from comma_nav.engine import Engine, EngineError
from comma_nav.follower import Follower
import export_tracks
import route_plan_eval as plan_eval
from known_hard_curves import BASE, CEIL, CLUSTER_M, CURVE_K, GROUP_S, MIN_ROUTES, cluster, hav

MAPD = os.path.join(os.path.dirname(BASE.rstrip("/")), "mapd_log", "*.jsonl")
MIN_KPH, MAX_KPH = 10.0, 60.0
HOLDOFF_S, SAME_CURVE_S, SAME_CURVE_M = 20.0, 60.0, 30.0   # as curve_shadow
TURN_DEG, ROUNDABOUT = 45.0, ("roundabout", "exit roundabout")
MATCH_M, LEAD_S = 50.0, 45.0
SETS = (("map",), ("learned",), ("route",), ("map", "route"), ("map", "learned"), ("map", "learned", "route"))


def scan(prefix):
  """(hard moments as (t, fix, kph), fixes as (t, lat, lon, heading, kph, lat active), span) for one drive."""
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  st = {"v": 0.0, "press": False, "lat": False, "cmd": 0.0, "des": 0.0, "fix": None}
  was_pressed, first, last, wall0, raw, fixes = False, None, None, None, [], []
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
          fixes.append((t, st["fix"][0], st["fix"][1], math.degrees(loc.calibratedOrientationNED.value[2]) % 360, st["v"] * 3.6, st["lat"]))
      elif w == "controlsState" and st["lat"] and not st["press"]:
        cs = m.controlsState; kph = st["v"] * 3.6
        des, act = abs(cs.desiredCurvature), abs(cs.curvature)
        st["des"] = des
        if kph < MIN_KPH or des <= CURVE_K or not st["fix"]:
          continue
        if abs(st["cmd"]) >= 0.98 * float(np.interp(st["v"], *CEIL)) or act < 0.75 * des:
          raw.append((t, st["fix"], kph))
  moments, lt = [], -1e18
  for t, fix, kph in sorted(raw):
    if t - lt > GROUP_S * 1e9:
      moments.append((t, fix, kph))
    lt = t
  return moments, fixes, (first, last, wall0)


def load_mapd(span):
  first, last, wall0 = span
  wall1 = wall0 + (last - first) / 1e9
  recs = []
  for f in glob.glob(MAPD):
    started = calendar.timegm(time.strptime(os.path.basename(f)[:20], "%Y-%m-%d--%H-%M-%S"))
    if not wall0 - 120 <= started <= wall1:
      continue
    for line in open(f):
      try:
        r = json.loads(line)
      except ValueError:
        continue
      if first <= r.get("mono_ns", 0) <= last:
        recs.append((r["mono_ns"], r.get("MapCurvatures") or []))
  return sorted(recs, key=lambda r: r[0])


def replay(fixes, mapd, known, bundle, rules, holdoff_s=HOLDOFF_S):
  """[(t, target fix)] for the would-be warnings of this set of rules."""
  follower = Follower(bundle or {"routes": []})
  mapd_t = [r[0] for r in mapd]
  out, last_t, last_fix = [], -1e18, None
  for t, lat, lon, heading, kph, lat_active in fixes:
    if last_fix is not None and hav((lat, lon), last_fix) < 2.0:
      continue
    last_fix = (lat, lon)
    state = follower.update(lat, lon, heading) if "route" in rules else None
    if not lat_active or not MIN_KPH <= kph <= MAX_KPH:
      continue
    hits = []
    if "map" in rules and mapd:
      i = bisect.bisect_right(mapd_t, t) - 1
      hit = tight_curve_ahead(lat, lon, heading, mapd[i][1]) if i >= 0 else None
      if hit is not None:
        hits.append((hit.distance, (hit.latitude, hit.longitude)))
    if "learned" in rules:
      hit = known_curve_ahead(lat, lon, heading, known)
      if hit is not None:
        hits.append((hit.distance, (hit.latitude, hit.longitude)))
    if state is not None and state["state"] == "following":
      for n in state["ahead"]:
        if n["distance_m"] <= LOOKAHEAD_M and (abs(n["turn_deg"]) >= TURN_DEG or n["type"] in ROUNDABOUT):
          hits.append((n["distance_m"], (n["lat"], n["lon"])))
          break
    if not hits:
      continue
    target = min(hits)[1]
    if t - last_t < holdoff_s * 1e9:
      continue
    if any(t - wt < SAME_CURVE_S * 1e9 and hav(target, pt) < SAME_CURVE_M for wt, pt in out[-8:]):
      continue
    out.append((t, target)); last_t = t
  return out


def score(moments, warns):
  """(moments caught, warnings that caught nothing, seconds of lead for each caught moment)."""
  used, leads = set(), []
  for t, fix, _ in moments:
    hits = [k for k, (wt, target) in enumerate(warns) if t - LEAD_S * 1e9 <= wt <= t and hav(target, fix) <= MATCH_M]
    used.update(hits)
    if hits:
      leads.append((t - warns[hits[0]][0]) / 1e9)
  return len(leads), len(warns) - len(used), leads


def main():
  args = sys.argv[1:]
  holdoff_s = HOLDOFF_S
  use_history = "--no-history" not in args
  all_drives = "--all-drives" in args
  args = [a for a in args if a not in ("--no-history", "--all-drives")]
  if "--holdoff" in args:
    k = args.index("--holdoff")
    holdoff_s = float(args[k + 1]); del args[k:k + 2]
  engine = Engine(plan_eval.NAV_DATA)
  found = export_tracks.drives()
  drives = {}
  for prefix in args:
    moments, fixes, span = scan(prefix)
    if span[0] is None or span[2] is None or len(fixes) < 100:
      continue
    drives[prefix] = (moments, fixes, span)
  tracks = {p: export_tracks.load(p, found) for p in drives}
  names = {p: export_tracks.name(p, found) for p in drives}
  thinned = {names[p]: nav_history.thin(tracks[p]) for p in sorted(drives, key=names.get) if len(tracks[p]) >= 100}

  whole = plan_eval.whole_drives(engine, thinned) if all_drives else None
  tot = {s: defaultdict(float) for s in SETS}
  leads = {s: [] for s in SETS}
  print(f"holdoff {holdoff_s:.0f} s, " + ("bundle of every other drive, no destination" if all_drives else
        f"routes {'with' if use_history else 'without'} the driver's past ways"))
  print("drive       moments | caught / false warnings for: " + "   ".join("+".join(s) for s in SETS))
  for prefix, (moments, fixes, span) in drives.items():
    mapd = load_mapd(span)
    if not any(curv for _, curv in mapd):
      print(f"{prefix}: no mapd log, skipped"); continue
    others = [(fix, r) for r, (ms, _, _) in drives.items() if r != prefix for _, fix, _ in ms]
    known = [{"latitude": c["centre"][0], "longitude": c["centre"][1]} for c in cluster(others) if len(c["routes"]) >= MIN_ROUTES]
    bundle = None
    trip_legs = plan_eval.legs(tracks[prefix]) if len(tracks[prefix]) >= 100 else []
    if all_drives:
      bundle = {"routes": plan_eval.history_routes(engine, names.get(prefix), [], thinned, whole)}
    elif trip_legs:
      try:
        own = plan_eval.history_routes(engine, names[prefix], trip_legs, thinned) if use_history else []
        bundle = plan_eval.plan(engine, trip_legs, 2, own)
      except EngineError:
        pass
    line = f"{prefix}  {len(moments):5d}   |"
    for s in SETS:
      warns = replay(fixes, mapd, known, bundle, s, holdoff_s)
      caught, false, ls = score(moments, warns)
      t = tot[s]
      t["moments"] += len(moments); t["caught"] += caught; t["warnings"] += len(warns); t["false"] += false; t["drives"] += 1
      leads[s] += ls
      line += f"  {caught:2d}/{false:2d}"
    print(line)

  print()
  for s in SETS:
    t = tot[s]
    m, w = max(t["moments"], 1), max(t["warnings"], 1)
    lead = f", median lead {np.median(leads[s]):.0f} s" if leads[s] else ""
    print(f"{'+'.join(s):20s} caught {int(t['caught']):3d} of {int(t['moments'])} moments ({100 * t['caught'] / m:.0f}%), "
          f"{int(t['warnings'])} warnings, {int(t['false'])} false ({100 * t['false'] / w:.0f}%){lead}, {int(t['drives'])} drives")


if __name__ == "__main__":
  main()
