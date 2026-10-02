"""Would a route planned in advance have announced this drive's turns?

For every drive it plans a route from where the drive started to where it ended (a round trip is
split at its farthest point into an outbound and a return route), replays the drive through
comma-nav's route follower, and scores the maneuvers the follower announced against the turns the
car really made. The destination is read off the drive itself, so this is the best case for "I told
it where I am going"; the route is the planner's own choice, not the one that was driven.

Turns are turn_memory's: a heading swing of TURN_DEG within TURN_WINDOW_M. A maneuver is announced
when the follower is on the route, the maneuver is within LOOKAHEAD_M and it counts as a turn: its
heading change is at least TURN_DEG, or it is a roundabout. Reported per drive, as in turn_memory:
turns predicted, false alarms (announced maneuvers no turn followed) and lead distance, plus the
share of the drive spent on a planned route. The last line repeats the totals without the
roundabout clause.

With --history the bundle also carries the driver's own ways between the same two places, built by
comma-nav's history module from the other drives. `first` puts those ahead of the planner's routes,
`only` drops the planner's. `all` takes no origin or destination at all: the bundle is every
other drive, whole, map-matched into routes. The drive being scored never contributes to itself,
so this is leave-one-out.

The follower's `next` is the maneuver ahead on the one route it follows. With --any-route the
score reads its `ahead` instead: the maneuver ahead on every route the car is on, which is what
announcing every possible turn does where the bundle's routes part ways.

Usage: python mapd/route_plan_eval.py [--alternates N] [--history first|only|all] [--any-route] <route_id> [<route_id> ...]
Needs comma-nav installed in the environment and its routing tiles built (COMMA_NAV_DATA).
"""
import os, sys
from bisect import bisect_right
from collections import Counter
import numpy as np
from comma_nav import bundle as nav_bundle, history as nav_history
from comma_nav.engine import Engine, EngineError
from comma_nav.follower import Follower
import export_tracks
from turn_memory import LOOKAHEAD_M, SAME_TURN_M, TURN_DEG, hav, turns

NAV_DATA = os.environ.get("COMMA_NAV_DATA", os.path.expanduser("~/Projects/comma-nav/data"))
ROUND_TRIP_M = 300.0   # a drive that ends this close to its start went somewhere and came back
STEP_M = 5.0
TURN_SLACK_M = 60.0    # driven distance allowed between where a maneuver was expected and the turn
SAME_MANEUVER_M = 3.0  # alternates share their first streets, so the same maneuver comes from several routes
ROUNDABOUT = ("roundabout", "exit roundabout")
RULES = {"turn or roundabout": lambda n: abs(n["turn_deg"]) >= TURN_DEG or n["type"] in ROUNDABOUT,
         "turn only": lambda n: abs(n["turn_deg"]) >= TURN_DEG}


def legs(tr):
  start, end = tr[0][:2], tr[-1][:2]
  if hav(start, end) >= ROUND_TRIP_M:
    return [(start, end)]
  far = max(tr, key=lambda p: hav(start, p[:2]))[:2]
  return [(start, far), (far, end)] if hav(start, far) >= ROUND_TRIP_M else []


def whole_drives(engine, thinned):
  """{drive name: its routes}, each drive map-matched whole, with no origin or destination."""
  return {name: nav_history.routes_from_drive(engine, track, "d" + name.replace("--", "-"), f"drive {name}")
          for name, track in thinned.items()}


def history_routes(engine, own_name, trip_legs, thinned, whole=None):
  """The same legs as the other drives drove them, or with `whole` every other drive entire."""
  if whole is not None:
    return [r for name, rs in whole.items() if name != own_name for r in rs]
  routes = []
  for n, (a, b) in enumerate(trip_legs):
    routes += nav_history.routes_between(engine, thinned, a, b, f"leg{n}", f"leg {n}", skip=(own_name,))
  return routes


def plan(engine, trip_legs, alternates, history=()):
  routes = list(history)
  for n, (a, b) in enumerate(trip_legs):
    for i, trip in enumerate(engine.routes(a, b, alternates) if alternates >= 0 else []):
      routes.append(nav_bundle.to_route(trip, f"leg{n}" + (f"-alt{i}" if i else ""), f"leg {n} alt {i}"))
  return nav_bundle.build_bundle(routes, 12) if routes else None


def replay(tr, bundle):
  """[(fix index, distance driven so far, follower state)] for one fix every STEP_M."""
  follower, out, last, driven = Follower(bundle or {"routes": []}), [], None, 0.0
  for idx, fix in enumerate(tr):
    if last is not None:
      step = hav(fix[:2], last)
      if step < STEP_M:
        continue
      driven += step
    last = fix[:2]
    out.append((idx, driven, follower.update(fix[0], fix[1], fix[2])))
  return out


def on_route(state):
  return state["state"] in ("following", "arrived")


def announcements(samples, is_turn, any_route):
  """Each maneuver that would have been announced, once, with the lead it was first seen at."""
  out = []
  for _, driven, s in samples:
    if s["state"] != "following":
      continue
    for n in s["ahead"] if any_route else [s["next"]]:
      if n is None or n["distance_m"] > LOOKAHEAD_M or not is_turn(n):
        continue
      point, expected = (n["lat"], n["lon"]), driven + n["distance_m"]
      if not any(hav(point, a["point"]) < SAME_MANEUVER_M and abs(expected - a["expected"]) < TURN_SLACK_M for a in out):
        out.append({"point": point, "expected": expected, "lead": n["distance_m"]})
  return out


def score(tr, tns, samples, is_turn, any_route=False):
  """(lead of every predicted turn, false alarms, state at each missed turn)."""
  idxs = [s[0] for s in samples]
  at = lambda i: samples[max(0, bisect_right(idxs, i) - 1)]
  ann = announcements(samples, is_turn, any_route)
  used, leads, missed = set(), [], Counter()
  for t in tns:
    i, j = t[3]
    lo, hi = at(i)[1] - TURN_SLACK_M, at(j)[1] + TURN_SLACK_M
    path = [p[:2] for p in tr[i:j + 1:5]] + [tr[j][:2]]
    hits = [k for k, a in enumerate(ann) if lo <= a["expected"] <= hi and min(hav(a["point"], p) for p in path) <= SAME_TURN_M]
    used.update(hits)
    if hits:
      leads.append(ann[hits[0]]["lead"])
    else:
      missed["on route" if on_route(at(i)[2]) else "off route"] += 1
  return leads, len(ann) - len(used), missed


def main():
  args = sys.argv[1:]
  alternates, history = 0, None
  any_route = "--any-route" in args
  args = [a for a in args if a != "--any-route"]
  if "--alternates" in args:
    k = args.index("--alternates")
    alternates = int(args[k + 1]); del args[k:k + 2]
  if "--history" in args:
    k = args.index("--history")
    history = args[k + 1]; del args[k:k + 2]
  engine = Engine(NAV_DATA)

  found = export_tracks.drives()
  tracks = {p: tr for p in args if len(tr := export_tracks.load(p, found)) >= 100}
  names = {p: export_tracks.name(p, found) for p in tracks}
  thinned = {names[p]: nav_history.thin(tracks[p]) for p in sorted(tracks, key=names.get)} if history else {}

  whole = whole_drives(engine, thinned) if history == "all" else None
  rows = []
  groups = ("all drives", "drives with a history route", "drives without one") if history else ("all drives",)
  tot = {(name, g): Counter() for name in RULES for g in groups}
  all_leads = {key: [] for key in tot}
  for prefix, tr in tracks.items():
    trip_legs = legs(tr)
    if not trip_legs:
      print(f"{prefix}: never left the first {ROUND_TRIP_M:.0f} m, skipped"); continue
    try:
      own = history_routes(engine, names[prefix], trip_legs, thinned, whole) if history else []
      bundle = plan(engine, trip_legs, -1 if history in ("only", "all") else alternates, own)
    except EngineError as e:
      print(f"{prefix}: {e}, skipped"); continue
    tns = turns(tr)
    samples = replay(tr, bundle)
    driven = samples[-1][1]
    on = sum(b[1] - a[1] for a, b in zip(samples, samples[1:]) if on_route(b[2]))
    print(f"{prefix}: {len(tns)} turns, {len(trip_legs)} leg(s), {len(own)} history + "
          f"{len(bundle['routes']) - len(own) if bundle else 0} planned route(s), driven {driven / 1000:.1f} km")
    mine = ["all drives"] + ([groups[1] if own else groups[2]] if history else [])
    for name, is_turn in RULES.items():
      leads, alarms, missed = score(tr, tns, samples, is_turn, any_route)
      for g in mine:
        t = tot[(name, g)]
        t["drives"] += 1; t["turns"] += len(tns); t["hit"] += len(leads); t["alarms"] += alarms; t["on"] += on; t["driven"] += driven
        t.update({f"missed {k}": v for k, v in missed.items()})
        all_leads[(name, g)] += leads
      if name == "turn or roundabout":
        rows.append((prefix, len(tns), len(leads), alarms, leads, on / driven, len(own)))

  what = f"{alternates} alternates" if history not in ("only", "all") else "no planner routes"
  print(f"\nplanned route ({what}{', history ' + history if history else ''}{', any route' if any_route else ''})   turns  predicted  false alarms  median lead  on route  history")
  for prefix, n, hit, alarms, leads, share, hist in rows:
    med = f"{np.median(leads):5.0f} m" if leads else "    -  "
    print(f"  {prefix}  {n:5d}      {hit:5d}        {alarms:5d}      {med}     {100 * share:3.0f}%     {hist:3d}")
  for (name, g), t in tot.items():
    leads = all_leads[(name, g)]
    n = max(t["turns"], 1)
    tail = f", median lead {np.median(leads):.0f} m" if leads else ""
    print(f"\n{name}, {g} ({t['drives']}): {t['turns']} turns, predicted {t['hit']} ({100 * t['hit'] / n:.0f}%), {t['alarms']} false alarms{tail}, "
          f"on route {100 * t['on'] / max(t['driven'], 1):.0f}% of {t['driven'] / 1000:.0f} km; "
          f"missed {t['missed on route']} on route, {t['missed off route']} off route")


if __name__ == "__main__":
  main()
