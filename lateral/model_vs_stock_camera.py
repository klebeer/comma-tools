"""Lane position from the model versus what the stock Mazda camera would steer, per route.

Samples: openpilot steering, over 40 km/h, no driver torque, both lane lines prob > 0.7. The stock
camera still computes LKAS_REQUEST on bus 2 while openpilot steers; this shows when it wants torque.

Usage: python model_vs_stock_camera.py <route_id> [<route_id> ...]
"""
import os, sys, warnings
warnings.filterwarnings("ignore")
from opendbc.can.parser import CANParser
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"

def pct(xs, q):
  s = sorted(xs); return s[min(len(s) - 1, int(q * len(s)))] if s else float("nan")

for prefix in sys.argv[1:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  cam = CANParser("mazda_2017", [("CAM_LKAS", 0), ("CAM_LANEINFO", 0)], 2)
  v, lat, press, yl, yr, pl, pr = 0.0, False, False, None, None, 0, 0
  offs, req_when, req_all = [], {"left": [], "centre": [], "right": []}, []
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which()
      if w == "carState":
        v = m.carState.vEgo * 3.6; press = m.carState.steeringPressed
      elif w == "carControl":
        lat = m.carControl.latActive
      elif w == "modelV2" and len(m.modelV2.laneLines) >= 4:
        yl, yr = m.modelV2.laneLines[1].y[0], m.modelV2.laneLines[2].y[0]
        pl, pr = m.modelV2.laneLineProbs[1], m.modelV2.laneLineProbs[2]
      elif w == "can":
        cam.update([(m.logMonoTime, [(c.address, c.dat, c.src) for c in m.can])])
        if not (lat and v > 40 and not press and yl is not None and pl > 0.7 and pr > 0.7):
          continue
        req = cam.vl["CAM_LKAS"]["LKAS_REQUEST"]
        centre = (yl + yr) / 2          # + means the lane centre is right of the camera
        offs.append(centre)
        req_all.append(req)
        side = "left" if centre > 0.15 else "right" if centre < -0.15 else "centre"
        req_when[side].append(req)
  print(f"{prefix}: {len(offs)} samples (lateral active, >40 km/h, no driver torque, both lines >0.7)")
  if offs:
    print(f"  lane centre rel. camera (m, + = centre is to the right, car sits left): median {pct(offs, .5):+.3f}  p10 {pct(offs, .1):+.3f}  p90 {pct(offs, .9):+.3f}")
    nz = [r for r in req_all if r != 0]
    print(f"  stock camera LKAS_REQUEST non-zero in {100 * len(nz) / len(req_all):.1f}% of samples; median {pct(nz, .5) if nz else 0:+.0f}")
    for k, rs in req_when.items():
      nzk = [r for r in rs if r != 0]
      print(f"    car {k:6}: {len(rs):6d} samples, camera wants torque in {100 * len(nzk) / max(len(rs), 1):5.1f}%, mean request {sum(rs) / max(len(rs), 1):+7.1f}")
