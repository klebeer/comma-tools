"""Replays recorded drives through a route bundle file, as the device would follow it.

The bundle is the file comma-nav wrote. Each drive is fed fix by fix to comma-nav's route follower,
and every maneuver it would have announced (a turn of TURN_DEG or more, or a roundabout, within
LOOKAHEAD_M on any route the car is on) is checked against the turns the car really made. A bundle
holds the past drives themselves, so the routes that came from the drive being replayed are left
out first: otherwise the drive would just be following itself.

Reported per drive: turns announced, announcements no turn followed, lead distance and the share
of the drive spent on a route of the bundle. --timeline also lists, in order, what was announced
and which turns went unannounced.

Usage: python mapd/replay_bundle.py [--timeline] <bundle.json> <route_id> [<route_id> ...]
"""
import json, sys
from bisect import bisect_right
import numpy as np
from comma_nav import bundle as nav_bundle
from comma_nav.follower import usable
import export_tracks
import route_plan_eval as plan_eval
from turn_memory import LOOKAHEAD_M, SAME_TURN_M, hav, turns

IS_TURN = plan_eval.RULES["turn or roundabout"]


def announced(samples):
  """Each maneuver announced, once: where, when it was expected, its lead and what it said."""
  out = []
  for _, driven, s in samples:
    if s["state"] != "following":
      continue
    for n in s["ahead"]:
      if n["distance_m"] > LOOKAHEAD_M or not IS_TURN(n):
        continue
      point, expected = (n["lat"], n["lon"]), driven + n["distance_m"]
      if not any(hav(point, a["point"]) < plan_eval.SAME_MANEUVER_M and abs(expected - a["expected"]) < plan_eval.TURN_SLACK_M for a in out):
        what = " ".join(x for x in (n["type"], n["modifier"]) if x)
        out.append({"point": point, "expected": expected, "lead": n["distance_m"], "at": driven, "route": n["route_id"],
                    "text": f"{what} {n['turn_deg']:+.0f} deg" + (f" onto {n['street']}" if n["street"] else "")})
  return out


def judge(tr, samples, ann):
  """(events in driving order, leads of the turns announced). An event is (km, text)."""
  idxs = [s[0] for s in samples]
  at = lambda i: samples[max(0, bisect_right(idxs, i) - 1)]
  used, leads, events = set(), [], []
  for t in turns(tr):
    i, j = t[3]
    lo, hi = at(i)[1] - plan_eval.TURN_SLACK_M, at(j)[1] + plan_eval.TURN_SLACK_M
    path = [p[:2] for p in tr[i:j + 1:5]] + [tr[j][:2]]
    hits = [k for k, a in enumerate(ann) if lo <= a["expected"] <= hi and min(hav(a["point"], p) for p in path) <= SAME_TURN_M]
    used.update(hits)
    if hits:
      leads.append(ann[hits[0]]["lead"])
    else:
      side = "right" if t[2] > 0 else "left"
      where = "on a route, nothing there" if plan_eval.on_route(at(i)[2]) else "off every route"
      events.append((at(i)[1], f"        turned {side}, not announced ({where})"))
  for k, a in enumerate(ann):
    outcome = "turned" if k in used else "NO TURN FOLLOWED"
    events.append((a["at"], f"announced {a['lead']:3.0f} m ahead: {a['text']}  -> {outcome}"))
  return sorted(events), leads, len(ann) - len(used)


def main():
  args = sys.argv[1:]
  timeline = "--timeline" in args
  args = [a for a in args if a != "--timeline"]
  with open(args[0]) as f:
    bundle = json.load(f)
  nav_bundle.validate(bundle)
  planned = sum(not r["route_id"].startswith("d0") for r in bundle["routes"])
  print(f"bundle {bundle['bundle_id']}: {len(bundle['routes'])} routes ({planned} planned, {len(bundle['routes']) - planned} from past drives), "
        f"expires {bundle['expires_at']}, {'usable' if usable(bundle) else 'NOT usable'} by the follower")

  found = export_tracks.drives()
  tot = {"turns": 0, "hit": 0, "false": 0, "on": 0.0, "driven": 0.0}
  all_leads, rows = [], []
  for prefix in args[1:]:
    tr = export_tracks.load(prefix, found)
    if len(tr) < 100:
      continue
    name = export_tracks.name(prefix, found)
    own = "d" + nav_bundle.slug(name, 40)
    routes = [r for r in bundle["routes"] if not r["route_id"].startswith(own)]
    samples = plan_eval.replay(tr, {"routes": routes})
    events, leads, false = judge(tr, samples, announced(samples))
    driven = samples[-1][1]
    on = sum(b[1] - a[1] for a, b in zip(samples, samples[1:]) if plan_eval.on_route(b[2]))
    n = len(turns(tr))
    tot["turns"] += n; tot["hit"] += len(leads); tot["false"] += false; tot["on"] += on; tot["driven"] += driven
    all_leads += leads
    rows.append((name, n, len(leads), false, leads, on / max(driven, 1), driven, len(bundle["routes"]) - len(routes)))
    if timeline:
      print(f"\n{name}: {driven / 1000:.1f} km")
      for km, text in events:
        print(f"  {km / 1000:5.2f} km  {text}")

  print("\ndrive                   km  turns  announced  no turn followed  median lead  on a route  own routes left out")
  for name, n, hit, false, leads, share, driven, left in rows:
    med = f"{np.median(leads):4.0f} m" if leads else "   -  "
    print(f"  {name}  {driven / 1000:5.1f}  {n:5d}      {hit:5d}             {false:5d}       {med}        {100 * share:3.0f}%  {left:5d}")
  t = max(tot["turns"], 1)
  lead = f", median lead {np.median(all_leads):.0f} m" if all_leads else ""
  print(f"\ntotal {tot['turns']} turns: announced {tot['hit']} ({100 * tot['hit'] / t:.0f}%), {tot['false']} announcements with no turn"
        f"{lead}, on a route {100 * tot['on'] / max(tot['driven'], 1):.0f}% of {tot['driven'] / 1000:.0f} km")


if __name__ == "__main__":
  main()
