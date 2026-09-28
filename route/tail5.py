"""Last N seconds of a route in 10 s blocks: speed, lane position, line and edge visibility, camera intent, road."""
import bisect, json, os, sys, warnings
warnings.filterwarnings("ignore")
from opendbc.can.parser import CANParser
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
prefix, last_s, replay = sys.argv[1], float(sys.argv[2]), sys.argv[3]
segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
roads = [(r["t"], r.get("RoadName")) for r in map(json.loads, open(replay))]
rt = [t for t, _ in roads]
cam = CANParser("mazda_2017", [("CAM_LKAS", 0)], 2)
t0 = None; rows = []; st = {"v": 0, "lat": 0, "press": 0}
for s in segs:
  for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
    w = m.which(); t0 = m.logMonoTime if t0 is None else t0; t = (m.logMonoTime - t0) / 1e9
    if w == "carState":
      st.update(v=m.carState.vEgo * 3.6, press=int(m.carState.steeringPressed), angle=m.carState.steeringAngleDeg,
                blk=("L" if m.carState.leftBlinker else "") + ("R" if m.carState.rightBlinker else ""))
    elif w == "carControl":
      st["lat"] = int(m.carControl.latActive)
    elif w == "can":
      cam.update([(m.logMonoTime, [(c.address, c.dat, c.src) for c in m.can])])
      st["cam"] = cam.vl["CAM_LKAS"]["LKAS_REQUEST"]
    elif w == "modelV2" and len(m.modelV2.laneLines) >= 4 and len(m.modelV2.roadEdges) >= 2:
      md = m.modelV2
      rows.append((t, st["v"], st["lat"], st["press"], st.get("blk", ""), (md.laneLines[1].y[0] + md.laneLines[2].y[0]) / 2,
                   md.laneLineProbs[1], md.laneLineProbs[2], md.roadEdges[0].y[0], md.roadEdges[1].y[0],
                   md.laneLines[2].y[0] - md.laneLines[1].y[0], st.get("cam", 0)))
end = rows[-1][0]
print(f"{prefix}: route ends t+{end:.0f}s, showing t+{end - last_s:.0f}..{end:.0f}")
print("  t      km/h lat% drv% blk | centre(+=car left)  pL   pR  | leftEdge rightEdge laneW | cam!=0% | road")
b0 = end - last_s
while b0 < end:
  b = [r for r in rows if b0 <= r[0] < b0 + 10]
  if b:
    n = len(b); med = lambda i: sorted(x[i] for x in b)[n // 2]
    road = roads[max(0, bisect.bisect_right(rt, b0 + 5) - 1)][1] if roads else ""
    blk = "".join(sorted({x[4] for x in b if x[4]})) or "-"
    print(f"  {b0:6.0f} {med(1):4.0f} {100 * sum(x[2] for x in b) / n:4.0f} {100 * sum(x[3] for x in b) / n:4.0f} {blk:3} | "
          f"{med(5):+6.2f}            {med(6):.2f} {med(7):.2f} | {med(8):+7.2f} {med(9):+8.2f} {med(10):5.2f} | {100 * sum(1 for x in b if x[11] != 0) / n:6.0f} | {road}")
  b0 += 10
