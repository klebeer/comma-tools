"""Timeline of lane lines, road edges and path after engagement, with the road name."""
import bisect, glob, json, os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
prefix, w0, w1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
live = []
for f in glob.glob("/data/media/0/mapd_log/*.jsonl"):
  for l in open(f):
    try: r = json.loads(l); live.append((r["mono_ns"], r.get("RoadName")))
    except ValueError: pass
live.sort(); lt = [x[0] for x in live]
segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
t0 = None; st = {"v": 0, "lat": 0, "press": 0, "des": 0.0}; last = -9
print("   t    km/h lat drv | leftEdge(std) leftLine  rightLine  rightEdge(std) | pL   pR  | laneW roadW | car pos in road (0=right edge) | path y@20m | road")
for s in segs:
  for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
    w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
    if t < w0: continue
    if t > w1: sys.exit(0)
    if w == "carState": st.update(v=m.carState.vEgo * 3.6, press=int(m.carState.steeringPressed))
    elif w == "carControl": st["lat"] = int(m.carControl.latActive)
    elif w == "modelV2" and t - last >= 1.0 and len(m.modelV2.laneLines) >= 4:
      last = t; md = m.modelV2
      le, re = md.roadEdges[0].y[0], md.roadEdges[1].y[0]
      ll, rl = md.laneLines[1].y[0], md.laneLines[2].y[0]
      xs = list(md.position.x); ys = list(md.position.y)
      i20 = min(range(len(xs)), key=lambda k: abs(xs[k] - 20)) if xs else 0
      i = bisect.bisect_right(lt, m.logMonoTime) - 1
      road = live[i][1] if i >= 0 and m.logMonoTime - lt[i] < 3e9 else ""
      width = re - le
      print(f"  {t:6.1f} {st['v']:4.0f}  {st['lat']}   {st['press']}  | {le:+6.2f} ({md.roadEdgeStds[0]:.2f})  {ll:+6.2f}    {rl:+6.2f}    {re:+6.2f} ({md.roadEdgeStds[1]:.2f}) | "
            f"{md.laneLineProbs[1]:.2f} {md.laneLineProbs[2]:.2f} | {rl - ll:4.1f}  {width:5.1f} |  {re / width if width > 0 else float('nan'):.2f}  | {ys[i20] if ys else 0:+.2f} | {road}")
