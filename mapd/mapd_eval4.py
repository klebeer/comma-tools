"""Fixed 1.09 m/s^2 versus the learned speed-dependent steering limit, as a map curve warning.

Truth for a position: steerSaturated within HORIZON_S, or the car's actual lateral
acceleration in that window above the learned limit at its speed. Lane-change windows dropped.
Usage: PYTHONPATH=/data:... python mapd_eval4.py <route>:<replay.jsonl> ...
"""
import bisect, json, sys
import numpy as np
from mapd_eval2 import HORIZON_S, load_route, predict

# learned bins at the end of route eb2f9b1e95: centre m/s, (1 - friction) * latAccelFactor
BINS = [6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0]
CAP = [2.09, 2.43, 2.06, 1.06, 1.17, 1.37, 1.57]
LANE = {"laneChange", "preLaneChangeLeft", "preLaneChangeRight", "laneChangeBlocked"}


def limit(v_ms):
  return float(np.interp(v_ms, BINS, CAP))


def main():
  rows = []
  for arg in sys.argv[1:]:
    prefix, replay = arg.split(":")
    lat_acc, events = load_route(prefix)
    la_t = [t for t, _ in lat_acc]
    reps = [json.loads(line) for line in open(replay)]
    step = (reps[-1]["t"] - reps[0]["t"]) / max(len(reps) - 1, 1)
    for r in reps:
      t, v = r["t"], r["v_kmh"]
      if v < 30:
        continue
      names = {n for te, n in events if t <= te <= t + HORIZON_S}
      if names & LANE:
        continue
      i0, i1 = bisect.bisect_left(la_t, t), bisect.bisect_right(la_t, t + HORIZON_S)
      actual = max((a for _, a in lat_acc[i0:i1]), default=0.0)
      lim = limit(v / 3.6)
      real = ("steerSaturated" in names) or actual > lim
      rows.append((prefix, t, step, v, predict(r), lim, real, "steerSaturated" in names))
  minutes = sum(r[2] for r in rows) / 60
  n_real = sum(1 for r in rows if r[6])
  print(f">= 30 km/h, no lane change: {len(rows)} positions, ~{minutes:.1f} min, {n_real} real (demand above the learned limit or saturated)")
  print("  alert rule                       | alerts | real | precision | recall | false per 10 min | saturated windows caught")
  sat = [r for r in rows if r[7]]
  for name, rule in (("fixed pred > 1.09", lambda r: r[4] > 1.09),
                     ("learned pred > limit(v)", lambda r: r[4] > r[5]),
                     ("learned pred > 0.8 * limit(v)", lambda r: r[4] > 0.8 * r[5]),
                     ("learned pred > 0.6 * limit(v)", lambda r: r[4] > 0.6 * r[5])):
    alerts = [r for r in rows if rule(r)]
    real = [r for r in alerts if r[6]]
    false = [r for r in alerts if not r[6]]
    prec = len(real) / len(alerts) if alerts else float("nan")
    rec = len(real) / n_real if n_real else float("nan")
    print(f"  {name:32} | {len(alerts):6d} | {len(real):4d} | {prec:9.2f} | {rec:6.2f} | {len(false) / minutes * 10:16.1f} | {sum(1 for r in sat if rule(r))}/{len(sat)}")
  print("  by speed band, learned pred > limit(v): band | positions | real | alerts | precision | recall")
  for lo, hi in ((30, 50), (50, 70), (70, 200)):
    b = [r for r in rows if lo <= r[3] < hi]
    a = [r for r in b if r[4] > r[5]]
    rr = [r for r in a if r[6]]
    nr = sum(1 for r in b if r[6])
    print(f"    {lo}-{hi} | {len(b):4d} | {nr:4d} | {len(a):4d} | {len(rr) / len(a) if a else float('nan'):5.2f} | {len(rr) / nr if nr else float('nan'):5.2f}")


if __name__ == "__main__":
  main()
