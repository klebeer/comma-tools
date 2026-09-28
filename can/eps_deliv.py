"""Commanded torque versus EPS motor torque and LKAS_BLOCK in a window."""
import os, sys, warnings
warnings.filterwarnings("ignore")
from opendbc.can.parser import CANParser
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
prefix, w0, w1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
cp = CANParser("mazda_2017", [("STEER_RATE", 0), ("STEER_TORQUE", 0)], 0)
t0 = None; st = {}; last = -9
print("   t    km/h angle  cmdCAN  motor  driver  LKAS_BLOCK lat")
for s in segs:
  for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
    w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
    if t < w0 - 1: continue
    if t > w1: break
    if w == "can":
      cp.update([(m.logMonoTime, [(c.address, c.dat, c.src) for c in m.can])])
    elif w == "carState":
      st.update(v=m.carState.vEgo * 3.6, ang=m.carState.steeringAngleDeg)
    elif w == "carOutput":
      st["cmd"] = m.carOutput.actuatorsOutput.torqueOutputCan
    elif w == "carControl":
      st["lat"] = int(m.carControl.latActive)
    if t >= w0 and t - last >= 0.5 and "cmd" in st and "v" in st:
      last = t
      print(f"  {t:6.1f} {st['v']:4.0f} {st['ang']:6.1f} {st['cmd']:6.0f} {cp.vl['STEER_TORQUE']['STEER_TORQUE_MOTOR']:6.0f} "
            f"{cp.vl['STEER_TORQUE']['STEER_TORQUE_SENSOR']:6.0f}      {int(cp.vl['STEER_RATE']['LKAS_BLOCK'])}      {st.get('lat', 0)}")
