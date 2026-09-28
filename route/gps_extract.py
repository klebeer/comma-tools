"""GPS track of a route, joined to the mapd_logger road names, as CSV plus a summary."""
import bisect, csv, glob, json, math, os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
prefix, out_csv = sys.argv[1], sys.argv[2]
segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))

mapd = []
for f in glob.glob("/data/media/0/mapd_log/*.jsonl"):
  for line in open(f):
    try:
      r = json.loads(line)
      mapd.append((r["mono_ns"], r.get("RoadName"), r.get("MapSpeedLimit")))
    except ValueError:
      pass
mapd.sort(); mt = [m[0] for m in mapd]

rows, t0, v = [], None, 0.0
for s in segs:
  for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
    w = m.which()
    t0 = m.logMonoTime if t0 is None else t0
    if w == "carState":
      v = m.carState.vEgo * 3.6
    elif w == "gpsLocationExternal":
      g = m.gpsLocationExternal
      if not g.hasFix:
        continue
      i = bisect.bisect_right(mt, m.logMonoTime) - 1
      road, limit = (mapd[i][1], mapd[i][2]) if i >= 0 and m.logMonoTime - mt[i] < 3e9 else (None, None)
      rows.append({"t_s": round((m.logMonoTime - t0) / 1e9, 2), "unix_ms": g.unixTimestampMillis, "lat": g.latitude,
                   "lon": g.longitude, "alt_m": round(g.altitude, 1), "gps_kmh": round(g.speed * 3.6, 1), "car_kmh": round(v, 1),
                   "bearing": round(g.bearingDeg, 1), "acc_m": round(g.horizontalAccuracy, 1), "road": road or "",
                   "osm_limit_kmh": round(float(limit) * 3.6) if limit else ""})

with open(out_csv, "w", newline="") as f:
  wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
  wr.writeheader(); wr.writerows(rows)

def hav(a, b):
  la1, lo1, la2, lo2 = map(math.radians, (a["lat"], a["lon"], b["lat"], b["lon"]))
  h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
  return 2 * 6371000 * math.asin(math.sqrt(h))
dist = sum(hav(a, b) for a, b in zip(rows, rows[1:]))
dur = rows[-1]["t_s"] - rows[0]["t_s"]
acc = sorted(r["acc_m"] for r in rows)
print(f"{prefix}: {len(rows)} fixes over {dur / 60:.1f} min, {dist / 1000:.2f} km, avg {dist / dur * 3.6:.1f} km/h, "
      f"max {max(r['car_kmh'] for r in rows):.0f} km/h, alt {min(r['alt_m'] for r in rows):.0f}-{max(r['alt_m'] for r in rows):.0f} m")
print(f"  start {rows[0]['lat']:.5f},{rows[0]['lon']:.5f}  end {rows[-1]['lat']:.5f},{rows[-1]['lon']:.5f}  "
      f"horizontal accuracy median {acc[len(acc) // 2]:.1f} m, p90 {acc[int(.9 * len(acc))]:.1f} m")
by_road = {}
for a, b in zip(rows, rows[1:]):
  k = a["road"] or "(sin nombre)"
  d = by_road.setdefault(k, [0.0, 0.0, 0.0])
  d[0] += b["t_s"] - a["t_s"]; d[1] += hav(a, b); d[2] = max(d[2], a["car_kmh"])
print("  por vía (min, km, max km/h):")
for k, (tt, dd, mx) in sorted(by_road.items(), key=lambda kv: -kv[1][0])[:12]:
  print(f"    {k:32} {tt / 60:5.1f} {dd / 1000:6.2f} {mx:5.0f}")
