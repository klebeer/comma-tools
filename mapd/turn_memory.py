"""Can the driver's own GPS history predict the next turn, with no map route and no connectivity?

Turns come from the track itself: heading swings by at least TURN_DEG over a window no longer than
TURN_WINDOW_M while moving, anchored where the swing starts with the heading carried into it.

The memory is the other routes' tracks, thinned to STEP_M, each with its turns placed by distance
along it. A prediction needs the car to be *following* one of those tracks, not merely near a
point: the current fix matches a track sample (MATCH_M, heading within APPROACH_DEG) and so does
the fix TRAIL_M earlier, at a lower index. The next turn is then read forward along that track.
With --repeats a turn must also appear on two or more routes, so one-off detours do not predict.

Scoring is leave-one-out. Reported per route: turns predicted, lead distance, and predictions no
turn followed, which is what a false alert would feel like.

Usage: python turn_memory.py [--repeats] <route_id> [<route_id> ...]
"""
import math, os, sys, warnings
from collections import defaultdict
warnings.filterwarnings("ignore")
import numpy as np
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
MIN_KPH = 3.0
TURN_DEG, TURN_WINDOW_M = 45.0, 60.0
STEP_M, TRAIL_M = 5.0, 100.0
LOOKAHEAD_M, APPROACH_DEG, MATCH_M, SAME_TURN_M = 120.0, 45.0, 25.0, 30.0


def hav(a, b):
  la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
  h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
  return 2 * 6371000 * math.asin(math.sqrt(h))


def ang_diff(a, b):
  return abs((a - b + 180) % 360 - 180)


def track(prefix):
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  kph = 0.0
  out = []
  for s in segs:
    p = os.path.join(BASE, s, "rlog.zst")
    if not os.path.exists(p):
      continue
    for m in LogReader(p):
      w = m.which()
      if w == "carState":
        kph = m.carState.vEgo * 3.6
      elif w == "liveLocationKalman":
        loc = m.liveLocationKalman
        if loc.positionGeodetic.valid and kph >= MIN_KPH:
          out.append((loc.positionGeodetic.value[0], loc.positionGeodetic.value[1],
                      math.degrees(loc.calibratedOrientationNED.value[2]) % 360))
  return out


def thin(tr, step=STEP_M):
  """[(lat, lon, heading, distance along the track)]"""
  out, run = [], 0.0
  for i, p in enumerate(tr):
    if i == 0:
      out.append((p[0], p[1], p[2], 0.0)); last = p; continue
    run += hav(p[:2], last[:2]); last = p
    if run - out[-1][3] >= step:
      out.append((p[0], p[1], p[2], run))
  return out


def turns(tr):
  out, i = [], 0
  while i < len(tr):
    j, dist = i, 0.0
    while j + 1 < len(tr) and dist < TURN_WINDOW_M:
      dist += hav(tr[j][:2], tr[j + 1][:2]); j += 1
      swing = (tr[j][2] - tr[i][2] + 180) % 360 - 180
      if abs(swing) >= TURN_DEG:
        out.append(((tr[i][0], tr[i][1]), tr[i][2], swing)); i = j; break
    else:
      i += 1; continue
    i += 1
  return out


def place_turns(thinned, tns):
  """Each turn as (distance along the thinned track, fix, approach heading)."""
  placed = []
  for fix, head, _ in tns:
    best = min(range(len(thinned)), key=lambda k: hav(fix, thinned[k][:2]), default=None)
    if best is not None and hav(fix, thinned[best][:2]) < MATCH_M:
      placed.append((thinned[best][3], fix, head))
  return sorted(placed)


def match_index(thinned, fix, heading, lo=0):
  """Index of the nearest track sample to this fix on a like heading, or None."""
  best, bd = None, MATCH_M
  for k in range(lo, len(thinned)):
    d = hav(fix, thinned[k][:2])
    if d < bd and ang_diff(heading, thinned[k][2]) <= APPROACH_DEG:
      best, bd = k, d
  return best


def main():
  args = sys.argv[1:]
  need_repeats = "--repeats" in args
  args = [a for a in args if a != "--repeats"]

  raw, thinned, tns = {}, {}, {}
  for prefix in args:
    tr = track(prefix)
    if len(tr) < 100:
      continue
    raw[prefix] = tr
    thinned[prefix] = thin(tr)
    tns[prefix] = turns(tr)
    print(f"{prefix}: {len(tr)} fixes -> {len(thinned[prefix])} samples, {len(tns[prefix])} turns")

  def repeated(fix, exclude):
    seen = sum(any(hav(fix, t[0]) < SAME_TURN_M for t in ts) for p, ts in tns.items() if p != exclude)
    return seen >= 2

  tot = defaultdict(int); leads = []
  print(f"\nleave-one-out{' (repeats only)' if need_repeats else ''}   turns  predicted  false alarms  median lead")
  for prefix, tr in raw.items():
    memory = {}
    for p in raw:
      if p == prefix:
        continue
      placed = [t for t in place_turns(thinned[p], tns[p]) if not need_repeats or repeated(t[1], prefix)]
      if placed:
        memory[p] = (thinned[p], placed)
    actual = tns[prefix]
    hit, hit_lead, alarms = set(), {}, set()
    walk, last, trail = [], None, []
    for fix in tr:
      p = (fix[0], fix[1])
      if last is not None and hav(p, last) < 10.0:
        continue
      last = p
      trail.append((p, fix[2]))
      # the fix about TRAIL_M back, for the "has been following this track" check
      back = None
      run = 0.0
      for k in range(len(trail) - 1, 0, -1):
        run += hav(trail[k][0], trail[k - 1][0])
        if run >= TRAIL_M:
          back = trail[k - 1]; break
      if back is None:
        continue
      pred = None
      for tr_thin, placed in memory.values():
        j = match_index(tr_thin, back[0], back[1])
        if j is None:
          continue
        i = match_index(tr_thin, p, fix[2], lo=j)
        if i is None or tr_thin[i][3] - tr_thin[j][3] < TRAIL_M * 0.6:
          continue
        here = tr_thin[i][3]
        nxt = next((t for t in placed if 1.0 < t[0] - here <= LOOKAHEAD_M), None)
        if nxt and (pred is None or nxt[0] - here < pred[0]):
          pred = (nxt[0] - here, nxt[1])
      if pred is None:
        continue
      d, mfix = pred
      near = [k for k, t in enumerate(actual) if hav(mfix, t[0]) < SAME_TURN_M and ang_diff(fix[2], t[1]) <= APPROACH_DEG]
      if near:
        k = near[0]
        if k not in hit:
          hit.add(k); hit_lead[k] = d
      else:
        alarms.add((round(mfix[0], 5), round(mfix[1], 5)))
    ls = [hit_lead[k] for k in hit]; leads += ls
    tot["turns"] += len(actual); tot["hit"] += len(hit); tot["alarms"] += len(alarms)
    med = f"{np.median(ls):5.0f} m" if ls else "    -"
    print(f"  {prefix}  {len(actual):5d}      {len(hit):5d}        {len(alarms):5d}      {med}")
  t = max(tot["turns"], 1)
  tail = f", median lead {np.median(leads):.0f} m" if leads else ""
  print(f"\ntotal {tot['turns']} turns: predicted {tot['hit']} ({100 * tot['hit'] / t:.0f}%), {tot['alarms']} false alarms{tail}")


main()
