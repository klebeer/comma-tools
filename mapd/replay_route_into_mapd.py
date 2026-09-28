"""Replay a route's positions into mapd and record what it publishes for each one.

Run with mapd_manager paused, or it overwrites LastGPSPosition every second;
replay_route_into_mapd.sh does that. Writes one JSON line per position, every <step_s> seconds
of the route: time, position, bearing, speed and mapd's outputs for it.

Usage: python replay_route_into_mapd.py <route_prefix> <step_s> <out.jsonl>
"""
import json
import math
import os
import sys
import time
import warnings

warnings.filterwarnings("ignore")
from openpilot.common.params import Params
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
D = "/dev/shm/params/d/"
KEYS = ["MapCurvatures", "MapTargetVelocities", "MapSpeedLimit", "RoadName"]


def rd(k):
  try:
    return open(D + k).read()
  except OSError:
    return None


def mtime():
  try:
    return os.stat(D + "MapCurvatures").st_mtime_ns
  except OSError:
    return 0


def wait_updates(n, limit=4.0):
  seen, last, start = 0, mtime(), time.monotonic()
  while seen < n and time.monotonic() - start < limit:
    time.sleep(0.05)
    m = mtime()
    if m != last:
      seen, last = seen + 1, m
  return seen


def main():
  prefix, step, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  mem = Params("/dev/shm/params")

  points, t0, next_t, v = [], None, 0.0, 0.0
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which()
      t0 = m.logMonoTime if t0 is None else t0
      rel = (m.logMonoTime - t0) / 1e9
      if w == "carState":
        v = m.carState.vEgo * 3.6
      elif w == "liveLocationKalman" and rel >= next_t:
        loc = m.liveLocationKalman
        if loc.positionGeodetic.valid:
          points.append((rel, loc.positionGeodetic.value[0], loc.positionGeodetic.value[1],
                         math.degrees(loc.calibratedOrientationNED.value[2]), v))
          next_t = rel + step
  print(f"{prefix}: {len(points)} positions every {step}s", flush=True)

  with open(out, "w") as f:
    for i, (rel, lat, lon, bearing, v) in enumerate(points):
      mem.put("LastGPSPosition", json.dumps({"latitude": lat, "longitude": lon, "bearing": bearing}))
      updates = wait_updates(2)
      rec = {"t": round(rel, 1), "lat": lat, "lon": lon, "bearing": round(bearing, 1), "v_kmh": round(v, 1), "updates": updates}
      for k in KEYS:
        raw = rd(k)
        try:
          rec[k] = json.loads(raw) if raw else None
        except ValueError:
          rec[k] = raw
      f.write(json.dumps(rec) + "\n")
      if i % 25 == 0:
        print(f"  {i}/{len(points)} t+{rel:.0f}s road={rec.get('RoadName')!r} curv_points={len(rec.get('MapCurvatures') or [])}", flush=True)
  print("done", flush=True)


if __name__ == "__main__":
  main()
