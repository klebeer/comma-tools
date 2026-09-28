"""Where curve tracking error comes from: EPS ceiling, slew limit, EPS delivery, or controller room.

Every 10 ms sample with openpilot steering, no driver torque for 2 s, and a requested curvature
of at least 0.002 1/m is compared against the request ~0.34 s earlier. Samples reaching less than
0.75 of it are classified by the first cause that applies:
  ceiling   commanded CAN torque within 2% of the speed-dependent EPS ceiling
  slew      command rose by the full 12-count step in the direction of the turn in the last 50 ms
  delivery  the EPS reports applying under 0.6 of the command (STEER_RATE.LKAS_EFFECTIVE) or blocks it
  room      none of the above: the controller had authority left and did not use it
Driver takeovers in curves are classified by how well the car was tracking in the second before.
"""
import os, sys, warnings
from collections import defaultdict, deque
warnings.filterwarnings("ignore")
import numpy as np
from opendbc.can.parser import CANParser
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
CEIL = ([8.0, 8.5, 9.4, 10.3, 11.2, 12.1, 13.0, 13.9, 14.5],
        [1148, 1132, 1092, 1048, 1012, 920, 808, 676, 620])
SPEEDS = [(10, 20), (20, 30), (30, 40), (40, 60), (60, 90)]
CURV = [(0.002, 0.01), (0.01, 0.02), (0.02, 0.2)]
LAG = 34


def segments(prefix):
  return sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix),
                key=lambda n: int(n.rsplit("--", 1)[1]))


def scan(prefix):
  cp = CANParser("mazda_2017", [("STEER_RATE", 0)], 0)
  st = {"v": 0.0, "pressed": False, "lat": False, "cmd": 0.0}
  des_hist = deque(maxlen=LAG + 1); cmd_hist = deque(maxlen=6); ratio_hist = deque(maxlen=100)
  samples, takeovers, delays = [], [], []
  n_eff = n_cmd = 0
  last_press = -99.0; was_pressed = False; t0 = None
  for s in segments(prefix):
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
      if w == "can":
        cp.update([(m.logMonoTime, [(c.address, c.dat, c.src) for c in m.can])])
      elif w == "carState":
        st["v"] = m.carState.vEgo; st["pressed"] = m.carState.steeringPressed
      elif w == "carControl":
        st["lat"] = m.carControl.latActive
      elif w == "carOutput":
        st["cmd"] = m.carOutput.actuatorsOutput.torqueOutputCan
      elif w == "liveDelay":
        delays.append(m.liveDelay.lateralDelay)
      elif w == "controlsState":
        cs = m.controlsState
        des_hist.append(cs.desiredCurvature); cmd_hist.append(st["cmd"])
        eff = cp.vl["STEER_RATE"]["LKAS_EFFECTIVE"]; blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] != 0
        kph = st["v"] * 3.6
        if st["lat"] and abs(st["cmd"]) > 150 and not st["pressed"]:
          n_cmd += 1; n_eff += abs(eff) >= 0.6 * abs(st["cmd"])
        pressed_now = st["pressed"] and st["lat"]
        if pressed_now and not was_pressed and len(des_hist) > LAG and abs(des_hist[-LAG - 1]) >= 0.005 and kph >= 10:
          takeovers.append((t, kph, list(ratio_hist)))
        was_pressed = pressed_now
        if st["pressed"]:
          last_press = t
        if not st["lat"] or t - last_press < 2.0 or len(des_hist) <= LAG or kph < 10:
          ratio_hist.clear()
          continue
        des = des_hist[0]; act = cs.curvature
        if abs(des) < 0.002:
          continue
        r = act / des
        ratio_hist.append(r)
        samples.append((t, kph, abs(des), r, st["cmd"], eff, blocked, list(cmd_hist), np.sign(des)))
  return samples, takeovers, delays, n_eff / max(n_cmd, 1)


def classify(sample):
  t, kph, des, r, cmd, eff, blocked, cmds, sgn = sample
  ceil = float(np.interp(kph / 3.6, *CEIL))
  if abs(cmd) >= 0.98 * ceil:
    return "ceiling"
  steps = [b - a for a, b in zip(cmds, cmds[1:])]
  if steps and max(sgn * d for d in steps) >= 11:
    return "slew"
  if blocked or (abs(cmd) > 150 and abs(eff) < 0.6 * abs(cmd)):
    return "delivery"
  return "room"


def main():
  for prefix in sys.argv[1:]:
    samples, takeovers, delays, delivered = scan(prefix)
    print(f"\n=== {prefix}: {len(samples)} samples in curves; commands >150 the EPS applied at >=0.6: {100 * delivered:.1f}%")
    if delays:
      print(f"  lagd lateralDelay: first {delays[0]:.3f} s, last {delays[-1]:.3f} s")
    cells = defaultdict(lambda: defaultdict(int))
    for smp in samples:
      sb = next((b for b in SPEEDS if b[0] <= smp[1] < b[1]), None)
      cb = next((b for b in CURV if b[0] <= smp[2] < b[1]), None)
      if not sb or not cb:
        continue
      c = cells[(sb, cb)]; c["n"] += 1
      if smp[3] < 0.75:
        c["under"] += 1; c[classify(smp)] += 1
      elif smp[3] > 1.25:
        c["over"] += 1
    print("  speed   curvature (radius)      n  under%  over% | of under: ceiling  slew  delivery  room")
    for sb in SPEEDS:
      for cb in CURV:
        c = cells.get((sb, cb))
        if not c or c["n"] < 100:
          continue
        u = max(c["under"], 1)
        rad = f"{1 / cb[1]:.0f}-{1 / cb[0]:.0f} m"
        print(f"  {sb[0]:>2}-{sb[1]:<2}  {cb[0]:.3f}-{cb[1]:.3f} ({rad:>9}) {c['n']:5d}  {100 * c['under'] / c['n']:5.1f}  {100 * c['over'] / c['n']:5.1f} |"
              f"   {100 * c['ceiling'] / u:5.1f}  {100 * c['slew'] / u:5.1f}  {100 * c['delivery'] / u:6.1f}  {100 * c['room'] / u:5.1f}")
    kinds = defaultdict(int)
    for t, kph, rh in takeovers:
      if len(rh) < 30:
        kinds["no clean second before"] += 1
      else:
        med = float(np.median(rh))
        kinds["was under-tracking (<0.75)" if med < 0.75 else "was over-tracking (>1.25)" if med > 1.25 else "was tracking fine"] += 1
    print(f"  takeovers in curves (>=0.005 1/m, >=10 km/h): {len(takeovers)}", dict(kinds))


main()
