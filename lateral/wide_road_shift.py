"""How far right the car would have to move to sit in the right half of a wide two-way road.

The existing context shift only acts when the right lane line is missing. On a street with no
centre line the model often reports both lines confidently and invents a lane near the middle of
the whole road, which leaves the car in the oncoming half. That case is detected here from the
road edges alone: the road is much wider than the lane the model found, and the car sits left of
the right half's centre.

Target is the middle between the road's centre and the right edge less CURB_MARGIN, the same shape
the existing rule uses against the left line. Reported per route: how often the case appears, the
shift it would need, and how much of that MAX_SHIFT would actually allow.

Usage: python wide_road_shift.py <route_id> [<route_id> ...]
"""
import os, sys, warnings
warnings.filterwarnings("ignore")
import numpy as np
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
MIN_KPH, MAX_KPH = 10.0, 50.0
WIDE_ROAD_M = 6.0      # a road this wide carries two directions
MAX_LANE_M = 4.5       # above this the model has merged lanes and the lane figure means nothing
MAX_EDGE_STD = 1.0
CURB_MARGIN = 0.8
MAX_SHIFT = 0.5        # what the deployed rule allows today


def main():
  for prefix in sys.argv[1:]:
    segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
    v, lat = 0.0, False
    seen = need = 0
    shifts, widths = [], []
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
        elif w == "modelV2" and lat and MIN_KPH <= v <= MAX_KPH:
          md = m.modelV2
          if len(md.laneLines) < 4 or len(md.roadEdges) < 2 or len(md.roadEdgeStds) < 2:
            continue
          seen += 1
          if max(md.roadEdgeStds[0], md.roadEdgeStds[1]) > MAX_EDGE_STD:
            continue
          # y is positive to the right: left edge negative, right edge positive
          le, re = md.roadEdges[0].y[0], md.roadEdges[1].y[0]
          road = re - le
          lane = md.laneLines[2].y[0] - md.laneLines[1].y[0]
          if road < WIDE_ROAD_M or lane > MAX_LANE_M or lane <= 0:
            continue
          middle = (le + re) / 2
          target = (middle + (re - CURB_MARGIN)) / 2   # centre of the right half, clear of the curb
          if target <= 0.05:                            # already right of it, nothing to do
            continue
          need += 1
          shifts.append(target); widths.append(road)
    print(f"\n=== {prefix}: {seen} samples steering at {MIN_KPH:.0f}-{MAX_KPH:.0f} km/h")
    if not shifts:
      print("  wide-road case never met the conditions")
      continue
    a = np.array(shifts)
    print(f"  case present in {need} samples ({100 * need / max(seen, 1):.0f}%), road width median {np.median(widths):.1f} m")
    print(f"  shift needed: median {np.median(a):.2f} m, p75 {np.percentile(a, 75):.2f} m, p90 {np.percentile(a, 90):.2f} m")
    print(f"  with MAX_SHIFT {MAX_SHIFT} m it would cover {100 * np.mean(np.minimum(a, MAX_SHIFT) / a):.0f}% of what is needed; "
          f"{100 * np.mean(a <= MAX_SHIFT):.0f}% of samples fully")


main()
