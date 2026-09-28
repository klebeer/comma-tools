"""Curvature tracking with openpilot steering and no driver torque, by speed and curve tightness.

ratio is achieved over requested curvature; "under" is below 0.75. "pinned" means the controller output
sat at the same value above 0.6, i.e. at the torque cap, so under-tracking NOT at the cap had torque to spare.

Usage: python curve_tracking_by_speed.py <route_id> [<route_id> ...]
"""
import os, sys, warnings
from collections import defaultdict, deque
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
SPEEDS = [(10, 20), (20, 30), (30, 40), (40, 60), (60, 90)]
CURV = [(0.002, 0.005), (0.005, 0.01), (0.01, 0.02), (0.02, 0.2)]
cells = defaultdict(lambda: {"n": 0, "ratio": [], "pinned": 0, "under": 0, "under_free": 0})
for prefix in sys.argv[1:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  st = {"v": 0.0, "pressed": False, "lat": False}; prev_out = 0.0; last_press = -99.0; t0 = None
  des_hist = deque(maxlen=40)   # actual curvature lags the request; compare against ~0.3 s earlier
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
      if w == "carState":
        st["v"] = m.carState.vEgo * 3.6; st["pressed"] = m.carState.steeringPressed
        if st["pressed"]:
          last_press = t
      elif w == "carControl":
        st["lat"] = m.carControl.latActive
      elif w == "controlsState":
        cs = m.controlsState; ls = cs.lateralControlState
        o = getattr(ls, ls.which()).output if ls.which() == "torqueState" else 0.0
        # The controller clips at a speed-dependent steer_max: an unchanged high output means it is clipped.
        pinned = abs(o) > 0.6 and abs(abs(o) - abs(prev_out)) < 1e-4; prev_out = o
        des_hist.append(cs.desiredCurvature)
        if not st["lat"] or t - last_press < 2.0 or len(des_hist) < 34:
          continue
        des = des_hist[-34]; act = cs.curvature
        if abs(des) < 0.002 or des * act < 0 and abs(act) > 0.001:
          continue
        sb = next((b for b in SPEEDS if b[0] <= st["v"] < b[1]), None)
        cb = next((b for b in CURV if b[0] <= abs(des) < b[1]), None)
        if not sb or not cb:
          continue
        c = cells[(sb, cb)]; r = abs(act) / abs(des)
        c["n"] += 1; c["ratio"].append(r); c["pinned"] += pinned
        if r < 0.75:
          c["under"] += 1; c["under_free"] += not pinned
print("only lateral active, no driver torque for 2 s; ratio = achieved / requested curvature (0.3 s lag)")
print("  speed    curvature (radius)     n    ratio med  under(<0.75)  of those NOT at torque cap  pinned%")
for sb in SPEEDS:
  for cb in CURV:
    c = cells.get((sb, cb))
    if not c or c["n"] < 100:
      continue
    rs = sorted(c["ratio"]); med = rs[len(rs) // 2]
    radius = f"{1 / cb[1]:.0f}-{1 / cb[0]:.0f} m"
    print(f"  {sb[0]:>2}-{sb[1]:<2}   {cb[0]:.3f}-{cb[1]:.3f} ({radius:>9})  {c['n']:5d}   {med:6.2f}      {100 * c['under'] / c['n']:5.1f}%"
          f"          {100 * c['under_free'] / max(c['under'], 1):5.1f}%             {100 * c['pinned'] / c['n']:5.1f}%")
