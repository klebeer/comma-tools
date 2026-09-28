"""Run lagd's own estimator over logged routes, with the stock and lower speed floors."""
import os, sys, warnings
from collections import Counter
warnings.filterwarnings("ignore")
import numpy as np
from opendbc.car.structs import car
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.constants import CV
from openpilot.selfdrive.locationd import lagd
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
SUBS = {"deviceMotion", "extrinsicsCalibration", "carState", "controlsState", "carControl"}
routes = sys.argv[1:]
_first = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == routes[0]), key=lambda n: int(n.rsplit("--", 1)[1]))[0]
CP = next(m.carParams for m in LogReader(os.path.join(BASE, _first, "rlog.zst")) if m.which() == "carParams")
def stream(prefix):
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      if m.which() in SUBS:
        yield m

for kmh in [float(x) for x in os.environ.get("FLOORS", "80.5,50,40").split(",")]:
  est = lagd.LateralLagEstimator(CP, 1 / 20.0, min_vego=kmh * CV.KPH_TO_MS)
  accepted, reasons, frame, n_points = [], Counter(), 0, 0
  orig = est.block_avg.update
  def rec(v, orig=orig):
    accepted.append(v); orig(v)
  est.block_avg.update = rec
  t_off = 0.0
  for prefix in routes:
    first = None
    for m in stream(prefix):
      first = m.logMonoTime if first is None else first
      t = t_off + (m.logMonoTime - first) * 1e-9
      est.handle_log(t, m.which(), getattr(m, m.which()))
      if m.which() == "deviceMotion":
        est.update_points()
        n_points += 1
        e = est
        if not e.lat_active: reasons["lat inactive"] += 1
        elif e.v_ego <= e.min_vego: reasons["below speed floor"] += 1
        elif e.steering_pressed: reasons["driver torque"] += 1
        elif e.steering_saturated: reasons["saturated"] += 1
        elif not e.calibrator.calib_valid: reasons["calibration"] += 1
        else: reasons["candidate"] += 1
        frame += 1
        if frame % 5 == 0:
          est.update_estimate()
    t_off = t + 600.0
  msg = est.get_msg(True).lateralDelay
  a = np.array(accepted)
  print(f"floor {kmh:5.1f} km/h: accepted estimates {len(a)}, validBlocks {msg.validBlocks}, calPerc {msg.calPerc}%, "
        f"status {msg.status}, delay in use {msg.lateralDelay:.3f}s, estimate {msg.lateralDelayEstimate:.3f}±{msg.lateralDelayEstimateStd:.3f}")
  if len(a):
    print(f"   individual estimates: median {np.median(a):.3f}s  p10 {np.percentile(a, 10):.3f}  p90 {np.percentile(a, 90):.3f}")
  tot = sum(reasons.values())
  print("   sample status: " + ", ".join(f"{k} {100 * v / tot:.0f}%" for k, v in reasons.most_common()))
print("CarParams steerActuatorDelay", round(CP.steerActuatorDelay, 3))
