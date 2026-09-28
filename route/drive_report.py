"""Lateral precision and alert report for one Mazda route.

Per speed band with lateral active: tracking error, saturation and driver press share. Then the
first and last learned torque parameters and the speed-dependent bins, sound, alert text and
onroadEvent transitions, and the hands-on and LDW warning bits as the camera sent them, as the car
saw CRZ_CTRL, and as openpilot sent them.

Usage: python drive_report.py <route_id>
"""
import collections
import os
import sys
import warnings

warnings.filterwarnings("ignore")

from opendbc.can.parser import CANParser
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
SPEED_BINS = [(-50, 23), (23, 40), (40, 50), (50, 60), (60, 70), (70, 80), (80, 200)]


def segments(prefix):
  names = [d for d in os.listdir(BASE) if d.startswith(prefix + "--") or ("--" in d and d.split("--")[1] == prefix)]
  names.sort(key=lambda n: int(n.rsplit("--", 1)[1]))
  return [os.path.join(BASE, n) for n in names]


def pct(xs, q):
  if not xs:
    return float("nan")
  s = sorted(xs)
  return s[min(len(s) - 1, int(q * len(s)))]


def edges(series):
  out, prev = [], object()
  for t, v in series:
    if v != prev:
      out.append((t, v))
      prev = v
  return out


def main():
  segs = segments(sys.argv[1])
  print(f"route {sys.argv[1]}: {len(segs)} segments")

  cam = CANParser("mazda_2017", [("CAM_LANEINFO", 0)], 2)
  car = CANParser("mazda_2017", [("CRZ_CTRL", 0)], 0)
  sent = CANParser("mazda_2017", [("CAM_LANEINFO", 0)], 0)

  t0 = None
  v = 0.0
  lat_active = False
  steering_pressed = False
  bins = collections.defaultdict(lambda: {"t": 0, "lat": 0, "err": [], "sat": 0, "press": 0})
  last_t = None
  torque_msgs = []
  torque_sp = None
  sounds, alerts, visual, events = [], [], collections.Counter(), collections.Counter()
  first_event = {}
  sig = collections.defaultdict(list)

  for seg in segs:
    p = os.path.join(seg, "rlog.zst")
    if not os.path.exists(p):
      continue
    for m in LogReader(p):
      w = m.which()
      t = m.logMonoTime / 1e9
      if t0 is None:
        t0 = t
      rel = t - t0

      if w == "carState":
        v = m.carState.vEgo * 3.6
        steering_pressed = m.carState.steeringPressed
      elif w == "carControl":
        lat_active = m.carControl.latActive
        visual[str(m.carControl.hudControl.visualAlert)] += 1
      elif w == "controlsState":
        # capped so a gap in the log is not counted as driving time
        dt = 0.0 if last_t is None else min(t - last_t, 0.1)
        last_t = t
        b = next(k for k in SPEED_BINS if k[0] <= v < k[1])
        s = bins[b]
        s["t"] += dt
        if lat_active:
          s["lat"] += dt
          ts = m.controlsState.lateralControlState.torqueState
          s["err"].append(abs(ts.error))
          s["sat"] += int(ts.saturated)
          s["press"] += int(steering_pressed)
      elif w == "lateralTorqueParameters":
        lt = m.lateralTorqueParameters
        torque_msgs.append((rel, lt.totalBucketPoints, lt.valid, lt.latAccelFactorFiltered,
                            lt.frictionCoefficientFiltered, lt.calPerc, lt.useParams))
      elif w == "customReserved19":
        c = m.customReserved19
        torque_sp = (list(c.speedBinCenters), list(c.speedBinLatAccelFactors), list(c.speedBinFrictions), list(c.speedBinValid))
      elif w == "selfdriveState":
        ss = m.selfdriveState
        sounds.append((rel, str(ss.alertSound)))
        alerts.append((rel, ss.alertText1))
      elif w == "onroadEvents":
        for e in m.onroadEvents:
          n = str(e.name)
          events[n] += 1
          first_event.setdefault(n, rel)
      elif w == "can":
        frames = [(c.address, c.dat, c.src) for c in m.can]
        cam.update([(m.logMonoTime, frames)])
        car.update([(m.logMonoTime, frames)])
        sig["cam HANDS_ON_STEER_WARN"].append((rel, int(cam.vl["CAM_LANEINFO"]["HANDS_ON_STEER_WARN"])))
        sig["cam LDW_WARN L/R"].append((rel, (int(cam.vl["CAM_LANEINFO"]["LDW_WARN_LL"]), int(cam.vl["CAM_LANEINFO"]["LDW_WARN_RL"]))))
        sig["car CRZ_CTRL HANDS_ON_STEER_WARN"].append((rel, int(car.vl["CRZ_CTRL"]["HANDS_ON_STEER_WARN"])))
        sig["car CRZ_CTRL HANDS_OFF_STEERING"].append((rel, int(car.vl["CRZ_CTRL"]["HANDS_OFF_STEERING"])))
      elif w == "sendcan":
        sent.update([(m.logMonoTime, [(c.address, c.dat, c.src) for c in m.sendcan])])
        sig["sent HANDS_ON_STEER_WARN"].append((rel, int(sent.vl["CAM_LANEINFO"]["HANDS_ON_STEER_WARN"])))
        sig["sent LDW_WARN L/R"].append((rel, (int(sent.vl["CAM_LANEINFO"]["LDW_WARN_LL"]), int(sent.vl["CAM_LANEINFO"]["LDW_WARN_RL"]))))

  total = sum(s["t"] for s in bins.values())
  print(f"duration {total:.0f} s\n")
  print("speed bin   time_s  lat_s  lat%   |err| mean  p90   sat%  pressed%")
  for b in SPEED_BINS:
    s = bins.get(b)
    if not s or s["t"] < 1:
      continue
    n = max(len(s["err"]), 1)
    mean = sum(s["err"]) / n if s["err"] else float("nan")
    print(f"{b[0]:>3}-{b[1]:<4}  {s['t']:7.0f} {s['lat']:6.0f} {100 * s['lat'] / s['t']:5.1f}   "
          f"{mean:8.3f} {pct(s['err'], 0.9):6.3f} {100 * s['sat'] / n:5.1f} {100 * s['press'] / n:7.1f}")

  if torque_msgs:
    f, l = torque_msgs[0], torque_msgs[-1]
    print("\nlateralTorqueParameters  (t, totalBucketPoints, valid, latAccelFactor, friction, calPerc, useParams)")
    print(f"  first {tuple(round(x, 3) if isinstance(x, float) else x for x in f)}")
    print(f"  last  {tuple(round(x, 3) if isinstance(x, float) else x for x in l)}")
  if torque_sp:
    print("\nspeed-dependent bins (center m/s, latAccelFactor, friction, valid)")
    for row in zip(*torque_sp):
      print(f"  {row[0]:5.1f}  {row[1]:6.3f}  {row[2]:6.3f}  {row[3]}")

  print("\nsounds (transitions)")
  for t, s in edges(sounds):
    if s != "none":
      print(f"  t+{t:7.1f}s {s}")
  print("\nalert text (transitions)")
  for t, a in edges(alerts):
    if a:
      print(f"  t+{t:7.1f}s {a}")
  print("\nvisualAlert counts:", dict(visual))
  print("\nonroadEvents")
  for n, c in events.most_common():
    print(f"  {n:34} x{c:<6} first t+{first_event[n]:.1f}s")
  print("\nCAN warning signals (transitions)")
  for k, series in sig.items():
    e = edges(series)
    print(f"  {k}: {len(e)} transitions")
    for t, val in e[:15]:
      print(f"     t+{t:7.1f}s -> {val}")


if __name__ == "__main__":
  main()
