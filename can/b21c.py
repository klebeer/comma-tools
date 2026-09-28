import os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
prefix = sys.argv[1]
bursts = [tuple(map(float, x.split("-"))) for x in sys.argv[2].split(",")]
segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
t0 = None; prev = None; ivals = []; start = None; tmax = 0; tq = []
for s in segs:
  for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
    t = m.logMonoTime / 1e9; t0 = t if t0 is None else t0; r = t - t0; tmax = r
    w = m.which()
    if w == "carControl":
      tq.append((r, abs(m.carControl.actuators.torque), m.carControl.latActive))
    if w != "can": continue
    for c in m.can:
      if c.src == 128 and c.address == 0x21C:
        b = (c.dat[4] >> 7) & 1
        if prev is not None and b != prev:
          if b: start = r
          elif start is not None: ivals.append((start, r)); start = None
        prev = b
if start is not None: ivals.append((start, tmax))
print(f"0x21C DBC bit39 high intervals: {len(ivals)}")
def maxtq(a, b):
  xs = [q for t, q, la in tq if a <= t <= b and la]
  return max(xs) if xs else 0.0
for a, b in ivals:
  hit = [f"{x:.1f}-{y:.1f}" for x, y in bursts if x < b + 0.5 and y > a - 0.5]
  print(f"  {a:7.2f}-{b:7.2f} ({b-a:5.2f}s)  max|torqueCmd| {maxtq(a, b):.2f}  beep {hit if hit else '-'}")
miss = [f"{x:.1f}-{y:.1f}" for x, y in bursts if not any(x < b + 0.5 and y > a - 0.5 for a, b in ivals)]
print("beeps with no bit39 interval:", miss)
