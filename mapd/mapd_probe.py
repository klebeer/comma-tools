"""Hand mapd one logged position and read back what it publishes."""
import json, math, os, sys, time, warnings
warnings.filterwarnings("ignore")
from openpilot.common.params import Params
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
prefix, seg, want_t = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
mem = Params("/dev/shm/params")
t0 = None; pick = None
for m in LogReader(os.path.join(BASE, f"{[d for d in os.listdir(BASE) if prefix in d][0].split('--')[0]}--{prefix}--{seg}", "rlog.zst")):
  if m.which() != "liveLocationKalman": continue
  l = m.liveLocationKalman
  if not l.positionGeodetic.valid: continue
  pick = (l.positionGeodetic.value[0], l.positionGeodetic.value[1], math.degrees(l.calibratedOrientationNED.value[2]))
  t0 = t0 or m.logMonoTime
  if (m.logMonoTime - t0) / 1e9 >= want_t: break
print("position", [round(x, 6) for x in pick])
keys = ["MapCurvatures", "MapTargetVelocities", "MapSpeedLimit", "RoadName", "MapAdvisoryLimit", "NextMapSpeedLimit"]
D = "/dev/shm/params/d/"
def rd(k):
  try:
    return open(D + k).read()
  except OSError:
    return None
before = {k: rd(k) for k in keys}
mem.put("LastGPSPosition", json.dumps({"latitude": pick[0], "longitude": pick[1], "bearing": pick[2]}))
start = time.monotonic()
while time.monotonic() - start < 15:
  now = {k: rd(k) for k in keys}
  if now["MapCurvatures"] and now != before:
    break
  time.sleep(0.2)
print(f"answered after {time.monotonic() - start:.1f}s")
for k in keys:
  print(f"{k}: {(rd(k) or '')[:700]}")
