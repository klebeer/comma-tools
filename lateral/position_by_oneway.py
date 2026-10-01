"""Where the car rides on two-way streets versus one-way carriageways, using the map's oneWay.

Each modelV2 sample with openpilot steering at 10-60 km/h is matched to the nearest map way (offline
mapd tiles, within MATCH_M). oneWay true means no oncoming traffic on that paved surface: a one-way
street, or one side of a divided road, which the map draws as two one-way ways. oneWay false means
both directions share the carriageway, so the car belongs in the right half.

Position is taken from the model's road edges (y positive to the right), which stay meaningful
where lane lines are faded or missing. Reported per class: road width, where the car sits relative
to the road's middle, and on two-way streets the share of time the car's centre was left of the
middle, which is riding in the oncoming half.

Usage: python position_by_oneway.py <route_id> [<route_id> ...]
"""
import glob, math, os, sys, warnings
from collections import defaultdict
warnings.filterwarnings("ignore")
import numpy as np
import capnp
from openpilot.tools.lib.logreader import LogReader

capnp.remove_import_hook()
SCHEMA = capnp.load("/data/offline_plain.capnp")
BASE = "/data/media/0/realdata"
TILES = "/data/media/0/osm/offline/*/*/-0.250000_-78.*"
MIN_KPH, MAX_KPH = 10.0, 60.0
MATCH_M, MAX_EDGE_STD, CELL = 12.0, 1.0, 0.001   # CELL in degrees, about 110 m


def seg_dist(p, a, b):
  kx = 111320 * math.cos(math.radians(p[0])); ky = 110540
  ax, ay = (a[1] - p[1]) * kx, (a[0] - p[0]) * ky
  bx, by = (b[1] - p[1]) * kx, (b[0] - p[0]) * ky
  dx, dy = bx - ax, by - ay
  t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, -(ax * dx + ay * dy) / (dx * dx + dy * dy)))
  return math.hypot(ax + t * dx, ay + t * dy)


def build_index():
  grid = defaultdict(list)
  for t in glob.glob(TILES):
    o = SCHEMA.Offline.from_bytes_packed(open(t, "rb").read(), traversal_limit_in_words=2**63 - 1)
    for w in o.ways:
      n = [(c.latitude, c.longitude) for c in w.nodes]
      for a, b in zip(n, n[1:]):
        for cell in {(int(a[0] / CELL), int(a[1] / CELL)), (int(b[0] / CELL), int(b[1] / CELL))}:
          grid[cell].append((a, b, w.oneWay, w.name))
  return grid


def match(grid, p):
  cx, cy = int(p[0] / CELL), int(p[1] / CELL)
  best = None
  for dx in (-1, 0, 1):
    for dy in (-1, 0, 1):
      for a, b, ow, name in grid.get((cx + dx, cy + dy), ()):
        d = seg_dist(p, a, b)
        if d < MATCH_M and (best is None or d < best[0]):
          best = (d, ow, name)
  return best


def main():
  grid = build_index()
  rows = defaultdict(list)   # oneWay -> (road width, car offset from the road middle, name)
  for prefix in sys.argv[1:]:
    segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
    v, lat, fix = 0.0, False, None
    for s in segs:
      p = os.path.join(BASE, s, "rlog.zst")
      if not os.path.exists(p):
        continue
      for m in LogReader(p):
        w = m.which()
        if w == "carState":
          v = m.carState.vEgo * 3.6
        elif w == "carControl":
          lat = m.carControl.latActive
        elif w == "liveLocationKalman":
          loc = m.liveLocationKalman
          if loc.positionGeodetic.valid:
            fix = (loc.positionGeodetic.value[0], loc.positionGeodetic.value[1])
        elif w == "modelV2" and lat and fix and MIN_KPH <= v <= MAX_KPH:
          md = m.modelV2
          if len(md.roadEdges) < 2 or len(md.roadEdgeStds) < 2:
            continue
          if max(md.roadEdgeStds[0], md.roadEdgeStds[1]) > MAX_EDGE_STD:
            continue
          hit = match(grid, fix)
          if hit is None:
            continue
          le, re = md.roadEdges[0].y[0], md.roadEdges[1].y[0]
          if re - le < 2.5:
            continue
          # (le + re) / 2 is where the road's middle lies from the car; positive means it is to the
          # right, so the car is left of the middle
          rows[hit[1]].append((re - le, (le + re) / 2, hit[2]))
    print(f"{prefix}: done")

  for ow, label in ((False, "two-way (shared carriageway)"), (True, "one-way carriageway")):
    r = rows.get(ow, [])
    if not r:
      print(f"\n{label}: no samples"); continue
    width = np.array([x[0] for x in r]); mid = np.array([x[1] for x in r])
    print(f"\n{label}: {len(r)} samples, road width median {np.median(width):.1f} m")
    print(f"  road middle from the car: median {np.median(mid):+.2f} m (positive = car is left of the middle)")
    if not ow:
      left = np.mean(mid > 0)
      print(f"  car centre in the oncoming half: {100 * left:.0f}% of the time; by more than 0.5 m: {100 * np.mean(mid > 0.5):.0f}%")
      worst = defaultdict(list)
      for wdt, md_, name in r:
        worst[name or "(unnamed)"].append(md_)
      print("  streets with the most time in the oncoming half:")
      ranked = sorted(((np.mean(np.array(x) > 0), len(x), n) for n, x in worst.items() if len(x) >= 40), reverse=True)[:8]
      for share, n, name in ranked:
        print(f"    {name:34} {100 * share:3.0f}% of {n} samples")


main()
