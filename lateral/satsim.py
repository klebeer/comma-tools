"""Replay openpilot's lateral saturation check with different rules over logged routes."""
import os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
VARIANTS = {"stock": (10.0, True), "low10": (10 / 3.6, True), "low10+cap": (10 / 3.6, False)}
for prefix in sys.argv[1:]:
  prev_out = 0.0
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  prev_out = 0.0
  st = {"v": 0.0, "pressed": False, "blk": "", "act": 0.0, "out": 0.0, "pid": 0.0, "lat": False}
  timer = None; t0 = None; last_t = None
  sim = {k: {"t": 0.0, "on": False, "ep": []} for k in VARIANTS}
  actual = []
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
      if w == "carParams" and timer is None:
        timer = m.carParams.steerLimitTimer
      elif w == "carState":
        cs = m.carState; st.update(v=cs.vEgo, pressed=cs.steeringPressed, blk=("L" if cs.leftBlinker else "") + ("R" if cs.rightBlinker else ""))
      elif w == "carOutput":
        st["out"] = m.carOutput.actuatorsOutput.torque
      elif w == "carControl":
        st["act"] = m.carControl.actuators.torque; st["lat"] = m.carControl.latActive
      elif w == "controlsState":
        ls = m.controlsState.lateralControlState
        st["pid"] = getattr(ls, ls.which()).output if ls.which() == "torqueState" else 0.0
        if getattr(ls, ls.which()).saturated and (not actual or t - actual[-1][0] > 5):
          actual.append((round(t, 1), round(st["v"] * 3.6)))
        dt = 0.01 if last_t is None else min(t - last_t, 0.05); last_t = t
        # the controller clips at a speed-dependent steer_max: pinned output at a high level is saturation
        sat_raw = st["lat"] and abs(st["pid"]) > 0.6 and abs(abs(st["pid"]) - abs(prev_out)) < 1e-4
        prev_out = st["pid"]
        limited = abs(st["act"] - st["out"]) > 1e-2
        for k, (vmin, respect_limit) in VARIANTS.items():
          s_ = sim[k]
          if sat_raw and st["v"] > vmin and not st["pressed"] and not (respect_limit and limited):
            s_["t"] += dt
          else:
            s_["t"] -= dt
          s_["t"] = min(max(s_["t"], 0.0), timer or 1.0)
          on = s_["t"] > (timer or 1.0) - 1e-3
          if on and not s_["on"]:
            s_["ep"].append((round(t, 1), round(st["v"] * 3.6), st["blk"] or "-"))
          s_["on"] = on
  dur = t / 60
  print(f"{prefix}: {dur:.1f} min, steerLimitTimer {timer}, real steerSaturated onsets {actual}")
  for k in VARIANTS:
    ep = sim[k]["ep"]
    print(f"   {k:10}: {len(ep):3d} alerts ({len(ep) / dur * 10:4.1f} per 10 min)  {ep[:10]}")
