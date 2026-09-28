"""False alarms of a map curve warning in a speed band, across replayed routes.

An alert fires at a replayed position when the map predicts more lateral acceleration ahead
than the threshold. It is real when, within the next HORIZON_S, the car actually exceeded the
steering limit or openpilot saturated. Lane-change windows are dropped (their lateral
acceleration is not the road's); driver-override windows stay, since the curve demands the
same acceleration whoever steers.

A false episode is a run of consecutive false alert positions. Imports
map_curve_warning_eval.py: copy it to the same directory.

Usage: python map_warning_false_alarms.py <route>:<replay.jsonl> ...
"""
import bisect
import json
import sys

from map_curve_warning_eval import HORIZON_S, load_route, predict

LIMIT = 1.09   # the tune's MAX_LAT_ACCEL for this car, m/s^2
BAND = (30.0, 50.0)
LANE = {"laneChange", "preLaneChangeLeft", "preLaneChangeRight", "laneChangeBlocked"}


def main():
  samples = []   # (route, t, step, v, pred, real)
  for arg in sys.argv[1:]:
    prefix, replay = arg.split(":")
    lat_acc, events = load_route(prefix)
    la_t = [t for t, _ in lat_acc]
    reps = [json.loads(line) for line in open(replay)]
    step = (reps[-1]["t"] - reps[0]["t"]) / max(len(reps) - 1, 1)
    for r in reps:
      t, v = r["t"], r["v_kmh"]
      if not (BAND[0] <= v < BAND[1]):
        continue
      names = {n for te, n in events if t <= te <= t + HORIZON_S}
      if names & LANE:
        continue
      i0, i1 = bisect.bisect_left(la_t, t), bisect.bisect_right(la_t, t + HORIZON_S)
      actual = max((a for _, a in lat_acc[i0:i1]), default=0.0)
      real = actual > LIMIT or "steerSaturated" in names
      samples.append((prefix, t, step, v, predict(r), real))

  minutes = sum(s[2] for s in samples) / 60
  n_real = sum(1 for s in samples if s[5])
  print(f"{BAND[0]:.0f}-{BAND[1]:.0f} km/h, no lane change: {len(samples)} positions, ~{minutes:.1f} min of driving, "
        f"{n_real} where the curve really exceeded {LIMIT} m/s^2")
  print("  threshold | alert positions | real | false | precision | recall | false episodes | false per 10 min")
  for th in (0.8, 1.09, 1.3, 1.6, 2.0):
    alerts = [s for s in samples if s[4] > th]
    real = [s for s in alerts if s[5]]
    false = [s for s in alerts if not s[5]]
    episodes, last = 0, None
    for s in false:
      key = (s[0], s[1])
      if last is None or last[0] != s[0] or s[1] - last[1] > s[2] * 1.5:
        episodes += 1
      last = key
    prec = len(real) / len(alerts) if alerts else float("nan")
    rec = len(real) / n_real if n_real else float("nan")
    rate = episodes / minutes * 10 if minutes else float("nan")
    print(f"  {th:9.2f} | {len(alerts):15d} | {len(real):4d} | {len(false):5d} | {prec:9.2f} | {rec:6.2f} | {episodes:14d} | {rate:16.1f}")


if __name__ == "__main__":
  main()
