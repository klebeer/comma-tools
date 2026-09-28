"""Post-drive check: card, panda mode, lateral, and the CRZ_CTRL relay on the wire."""
import os, sys, warnings
from collections import Counter
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
prefix = sys.argv[1]
segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
t0 = None; cp = None; ps = []; dead = []; ev = Counter(); first = {}; v = 0; lat_t = 0.0; last_t = None; lat = False
wire = Counter()   # 0x21C frames reaching the car (src 128) by (lateral allowed, bit39)
cam = Counter(); rej = Counter(); allowed = False; vmax = 0
for s in segs:
  for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
    w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
    if w == "carParams" and cp is None:
      c = m.carParams; cp = (str(c.carFingerprint), [(str(x.safetyModel), x.safetyParam) for x in c.safetyConfigs])
    elif w == "pandaStates" and len(m.pandaStates):
      p = m.pandaStates[0]; allowed = p.controlsAllowedLateral
      k = (str(p.safetyModel), p.safetyParam, p.safetyRxChecksInvalid, p.controlsAllowedLateral)
      if not ps or ps[-1][1] != k: ps.append((round(t, 1), k))
    elif w == "managerState":
      for pr in m.managerState.processes:
        if pr.shouldBeRunning and not pr.running and pr.name not in [d[1] for d in dead]:
          dead.append((round(t, 1), pr.name, pr.exitCode))
    elif w == "onroadEvents":
      for e in m.onroadEvents:
        n = str(e.name); ev[n] += 1; first.setdefault(n, round(t, 1))
    elif w == "carState":
      v = m.carState.vEgo * 3.6; vmax = max(vmax, v)
    elif w == "carControl":
      if last_t is not None and lat: lat_t += t - last_t
      lat = m.carControl.latActive; last_t = t
    elif w == "can":
      for f in m.can:
        if f.address != 0x21C: continue
        b39 = (f.dat[4] >> 7) & 1
        if f.src == 128: wire[(allowed, b39)] += 1
        elif f.src == 2: cam[(allowed, b39)] += 1
        elif f.src == 192: rej[allowed] += 1
dur = t
print(f"{prefix}: {dur / 60:.1f} min, max {vmax:.0f} km/h, lateral active {lat_t / 60:.1f} min")
print("carParams:", cp)
print("panda changes:", ps[:6], "..." if len(ps) > 6 else "")
print("dead processes:", dead or "none")
print("camera 0x21C (lateral allowed, bit39):", dict(cam))
print("0x21C reaching the car (lateral allowed, bit39):", dict(wire))
print("openpilot 0x21C rejected by panda (by lateral allowed):", dict(rej))
print("events:", ", ".join(f"{n} x{c}" for n, c in ev.most_common(12)))
