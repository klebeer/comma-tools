"""Does an NNLC model describe this car's steering torque better than a straight line, once scaled?

Samples: openpilot steering, no driver torque, command under the EPS ceiling and over 20 counts,
tracking within 0.15 m/s^2 of the request, lateral jerk request under 0.5 m/s^3, and at least
0.3 m/s^2 of lateral acceleration. The command that went out is the torque that curve needed. The
torque controller returns its output negated relative to its lateral accel, so needed = -command.

Per speed band, each predictor gets its own best scale (median needed / predicted), then its median
absolute error in CAN counts is compared:
  linear   k x lateral accel, the shape of the current torque feedforward
  nn:<m>   k x the sunnypilot NNLC model <m> at (v, lateral accel, 0 jerk, roll)
It also prints the scale each NN needs: NNLC trains on the stock 800-count scale, so a scale far
from 1 means the model would under- or over-steer this car as shipped.

Usage: python nnlc_offline_eval.py <route_id> [<route_id> ...]
"""
import os, sys, warnings
from collections import defaultdict
warnings.filterwarnings("ignore")
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.model import NNTorqueModel
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.helpers import TORQUE_NN_MODEL_PATH

BASE = "/data/media/0/realdata"
MODELS = ("MAZDA_CX5_2022", "MAZDA_CX9_2021")
NN_SCALE = 800.0
# opendbc EPS_CEILING_LOOKUP, m/s to CAN counts
CEIL = ([8.0, 8.5, 9.4, 10.3, 11.2, 12.1, 13.0, 13.9, 14.5], [1148, 1132, 1092, 1048, 1012, 920, 808, 676, 620])
BANDS = [(10, 30), (30, 50), (50, 70), (70, 130)]


def main():
  nn = {m: NNTorqueModel(os.path.join(TORQUE_NN_MODEL_PATH, m + ".json")) for m in MODELS}
  rows = defaultdict(list)  # band -> (needed, act, {nn name: prediction})
  for prefix in sys.argv[1:]:
    segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
    st = {"v": 0.0, "press": False, "lat": False, "cmd": 0.0, "roll": 0.0}
    n = 0
    for s in segs:
      p = os.path.join(BASE, s, "rlog.zst")
      if not os.path.exists(p):
        continue
      for m in LogReader(p):
        w = m.which()
        if w == "carState":
          st["v"] = m.carState.vEgo; st["press"] = m.carState.steeringPressed
        elif w == "carControl":
          st["lat"] = m.carControl.latActive
        elif w == "carOutput":
          st["cmd"] = m.carOutput.actuatorsOutput.torqueOutputCan
        elif w == "liveParameters":
          st["roll"] = m.liveParameters.roll
        elif w == "controlsState" and st["lat"] and not st["press"]:
          ls = m.controlsState.lateralControlState
          if ls.which() != "torqueState":
            continue
          ts = ls.torqueState; v = st["v"]; kph = v * 3.6
          des, act = ts.desiredLateralAccel, ts.actualLateralAccel
          if abs(act) < 0.3 or abs(des - act) > 0.15 or abs(ts.desiredLateralJerk) > 0.5:
            continue
          need = -st["cmd"]
          if abs(need) < 20 or abs(need) >= 0.98 * float(np.interp(v, *CEIL)) or np.sign(need) != np.sign(act):
            continue
          b = next((x for x in BANDS if x[0] <= kph < x[1]), None)
          if b is None:
            continue
          rows[b].append((need, act, {k: mdl.evaluate([v, act, 0.0, st["roll"]]) * NN_SCALE for k, mdl in nn.items()}))
          n += 1
    print(f"{prefix}: {n} samples")

  print("\nband km/h    n   predictor             scale   median |err| counts (after scaling)")
  for b in BANDS:
    r = rows[b]
    if len(r) < 30:
      continue
    need = np.array([x[0] for x in r]); act = np.array([x[1] for x in r])
    preds = {"linear": act}
    for k in MODELS:
      preds["nn:" + k] = np.array([x[2][k] for x in r])
    for k, pred in preds.items():
      ok = np.abs(pred) > 1e-6
      scale = float(np.median(need[ok] / pred[ok]))
      err = float(np.median(np.abs(scale * pred - need)))
      print(f"  {b[0]:>3}-{b[1]:<3} {len(r):5d}   {k:20} {scale:6.2f}   {err:6.0f}")


main()
