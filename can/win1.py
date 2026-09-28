import os, sys, warnings, collections
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
prefix, w0, w1, beep = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
t0 = None; prev = {}; edges = collections.defaultdict(list); last = -1
ev_seen = set()
for s in segs[: int(w1 // 60) + 1]:
  for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
    w = m.which(); t = m.logMonoTime / 1e9; t0 = t if t0 is None else t0; r = t - t0
    if not (w0 - 30 <= r <= w1):
      if w == "can":
        for c in m.can:
          if c.src in (0, 128): prev[(c.src, c.address)] = int.from_bytes(c.dat, "big")
      continue
    if w == "carState" and r >= w0 and r - last >= 0.5:
      cs = m.carState; last = r
      print(f"t+{r:6.1f} v={cs.vEgo*3.6:5.1f} angle={cs.steeringAngleDeg:7.1f} drvTorque={cs.steeringTorque:6.0f} epsTorque={cs.steeringTorqueEps:6.0f} pressed={int(cs.steeringPressed)} blk={'L' if cs.leftBlinker else ''}{'R' if cs.rightBlinker else ''} standstill={int(cs.standstill)}")
    elif w == "carControl" and r >= w0 and abs(r - last) < 0.02:
      cc = m.carControl
      print(f"          latActive={int(cc.latActive)} torqueCmd={cc.actuators.torque:+.2f} curvature={cc.actuators.curvature:+.4f}")
    elif w == "onroadEvents" and r >= w0:
      for e in m.onroadEvents:
        k = (str(e.name), int(r))
        if k not in ev_seen:
          ev_seen.add(k); print(f"   event t+{r:6.1f} {e.name}")
    elif w == "can":
      for c in m.can:
        if c.src not in (0, 128): continue
        key = (c.src, c.address); val = int.from_bytes(c.dat, "big"); n = len(c.dat) * 8
        p = prev.get(key)
        if p is not None and p != val:
          d = p ^ val
          while d:
            b = d & -d; edges[(key, n - b.bit_length())].append(round(r, 2)); d ^= b
        prev[key] = val
print(f"\nlow-activity bits (<= 6 edges in the whole window) with an edge within -2/+0.5 s of t+{beep}:")
for (key, bit), ts in sorted(edges.items(), key=lambda kv: (kv[0][0][1], kv[0][1])):
  if len(ts) <= 6 and any(beep - 2 <= t <= beep + 0.5 for t in ts):
    print(f"  src{key[0]:3d} 0x{key[1]:03X} bit{bit:2d} edges {ts}")
for (src, addr, bit) in ((0, 0x4FA, 33), (0, 0x4FA, 34), (128, 0x21C, 29), (128, 0x21C, 30), (128, 0x21C, 32)):
  print(f"  check src{src} 0x{addr:03X} bit{bit}: {edges.get(((src, addr), bit), [])[:20]}")
