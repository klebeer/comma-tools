"""Timeline of a window: desired vs actual curvature, torque, lane lines, road."""
import bisect, glob, json, os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
prefix, w0, w1, step = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
mapd = []
for f in glob.glob("/data/media/0/mapd_log/*.jsonl"):
  for l in open(f):
    try:
      r = json.loads(l)
    except ValueError:
      continue
    mapd.append((r["mono_ns"], r.get("RoadName")))
mapd.sort(); mt = [m[0] for m in mapd]
t0 = None; st = {}; last = -1e9; evs = []
print("  t     km/h  angle  drvTq  pressed lat torqueCmd sat | desCurv  actCurv | leftLine rightLine  pL   pR  laneW | road")
for s in segs:
  for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
    w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
    if t < w0 - 1: continue
    if t > w1: break
    if w == "carState":
      cs = m.carState; st.update(v=cs.vEgo, angle=cs.steeringAngleDeg, drv=cs.steeringTorque, pressed=int(cs.steeringPressed),
                                blk=("L" if cs.leftBlinker else "") + ("R" if cs.rightBlinker else ""))
    elif w == "carControl":
      st.update(lat=int(m.carControl.latActive))
    elif w == "carOutput":
      st.update(tq=m.carOutput.actuatorsOutput.torque)
    elif w == "controlsState":
      cs = m.controlsState; st.update(des=cs.desiredCurvature, act=cs.curvature,
                                      sat=int(getattr(cs.lateralControlState, cs.lateralControlState.which()).saturated))
    elif w == "modelV2" and len(m.modelV2.laneLines) >= 4:
      md = m.modelV2; st.update(yl=md.laneLines[1].y[0], yr=md.laneLines[2].y[0], pl=md.laneLineProbs[1], pr=md.laneLineProbs[2])
    elif w == "onroadEvents":
      for e in m.onroadEvents:
        n = str(e.name)
        if n in ("laneChange", "preLaneChangeLeft", "preLaneChangeRight", "steerSaturated", "steerOverride", "laneChangeBlocked") and (n, int(t)) not in evs:
          evs.append((n, int(t)))
    if t >= w0 and t - last >= step and "yl" in st and "des" in st:
      last = t
      i = bisect.bisect_right(mt, m.logMonoTime) - 1
      road = mapd[i][1] if i >= 0 and m.logMonoTime - mt[i] < 3e9 else ""
      print(f"  {t:6.1f} {st['v'] * 3.6:4.0f} {st['angle']:6.1f} {st['drv']:6.0f}    {st['pressed']}     {st.get('lat', 0)}   {st.get('tq', 0):+.2f}   {st['sat']}  | "
            f"{st['des']:+.4f}  {st['act']:+.4f} |  {st['yl']:+.2f}    {st['yr']:+.2f}   {st['pl']:.2f} {st['pr']:.2f} {st['yr'] - st['yl']:.2f} | {road} {st.get('blk', '')}")
print("events:", evs)
