"""Classify every driver takeover while openpilot steered, from the context just before it."""
import os, sys, warnings
from collections import Counter, deque
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
BASE = "/data/media/0/realdata"
LANE = {"laneChange", "preLaneChangeLeft", "preLaneChangeRight", "laneChangeBlocked"}

def classify(e):
  if e["blk"] or e["lanechg"]:
    return "cambio de carril / direccional"
  if e["v"] < 30 and abs(e["angle"]) > 60:
    return "giro cerrado < 30 km/h"
  if e["pinned"] or (abs(e["des"]) > 0.005 and abs(e["act"]) < 0.67 * abs(e["des"])):
    return "curva que no alcanza"
  if e["pl"] > 0.5 and e["pr"] > 0.5 and abs(e["off"]) > 0.25:
    return "corrección de posición en carril"
  if e["pl"] < 0.3 and e["pr"] < 0.3:
    return "sin líneas visibles"
  return "otro"

allrows = []
for prefix in sys.argv[1:]:
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  st = {"v": 0, "angle": 0, "pressed": False, "drv": 0, "blk": "", "lat": False, "cmd": 0.0, "des": 0.0, "act": 0.0, "out": 0.0,
        "off": 0.0, "pl": 0.0, "pr": 0.0}
  hist = deque(maxlen=200)   # ~2 s of controlsState snapshots
  t0 = None; in_ep = False; last_press = -99; lanechg_t = -99; prev_out = 0.0; eps = []
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which(); t0 = t0 or m.logMonoTime; t = (m.logMonoTime - t0) / 1e9
      if w == "carState":
        cs = m.carState
        st.update(v=cs.vEgo * 3.6, angle=cs.steeringAngleDeg, pressed=cs.steeringPressed, drv=cs.steeringTorque,
                  blk=("L" if cs.leftBlinker else "") + ("R" if cs.rightBlinker else ""))
        if st["pressed"] and st["lat"] and not in_ep and t - last_press > 1.0:
          pre = list(hist)
          des = sum(h[1] for h in pre) / max(len(pre), 1); act = sum(h[2] for h in pre) / max(len(pre), 1)
          pinned = sum(1 for h in pre if h[3]) > 0.3 * max(len(pre), 1)
          off = sorted(h[4] for h in pre)[len(pre) // 2] if pre else 0.0
          e = dict(route=prefix, t=round(t, 1), v=st["v"], angle=st["angle"], blk=st["blk"], lanechg=t - lanechg_t < 3,
                   des=des, act=act, pinned=pinned, off=off, pl=pre[-1][5] if pre else 0, pr=pre[-1][6] if pre else 0,
                   against=(st["drv"] * st["cmd"] < 0) and abs(st["cmd"]) > 0.1)
          e["cls"] = classify(e); e["peak"] = abs(st["drv"]); e["dur"] = 0.0; eps.append(e); in_ep = True
        if in_ep and eps:
          eps[-1]["peak"] = max(eps[-1]["peak"], abs(st["drv"]))
          if st["pressed"]:
            eps[-1]["dur"] = t - eps[-1]["t"]
        if st["pressed"]:
          last_press = t
        elif in_ep and t - last_press > 1.0:
          in_ep = False
      elif w == "carControl":
        st["lat"] = m.carControl.latActive; st["cmd"] = m.carControl.actuators.torque
      elif w == "onroadEvents":
        if any(str(x.name) in LANE for x in m.onroadEvents):
          lanechg_t = t
      elif w == "modelV2" and len(m.modelV2.laneLines) >= 4:
        md = m.modelV2
        st.update(off=(md.laneLines[1].y[0] + md.laneLines[2].y[0]) / 2, pl=md.laneLineProbs[1], pr=md.laneLineProbs[2])
      elif w == "controlsState":
        ls = m.controlsState.lateralControlState; o = getattr(ls, ls.which()).output if ls.which() == "torqueState" else 0.0
        pinned = abs(o) > 0.6 and abs(abs(o) - abs(prev_out)) < 1e-4; prev_out = o
        hist.append((t, m.controlsState.desiredCurvature, m.controlsState.curvature, pinned, st["off"], st["pl"], st["pr"]))
  dur = t / 60
  print(f"{prefix}: {dur:.1f} min, {len(eps)} takeovers ({len(eps) / dur * 10:.1f} per 10 min)")
  allrows += eps

print(f"\nall: {len(allrows)} takeovers")
cnt = Counter(e["cls"] for e in allrows)
for c, n in cnt.most_common():
  sub = [e for e in allrows if e["cls"] == c]
  vs = sorted(e["v"] for e in sub)
  ag = sum(1 for e in sub if e["against"])
  print(f"  {c:34} {n:3d} ({100 * n / len(allrows):4.0f}%)  speed median {vs[len(vs) // 2]:4.0f} km/h  against openpilot {ag}/{n}")
print("\nshort and light (< 0.5 s and peak driver torque < 40) vs deliberate, per class:")
for c in cnt:
  sub = [e for e in allrows if e["cls"] == c]
  light = sum(1 for e in sub if e["dur"] < 0.5 and e["peak"] < 40)
  durs = sorted(e["dur"] for e in sub); peaks = sorted(e["peak"] for e in sub)
  print(f"  {c:34} light {light:3d}/{len(sub):3d}  duration median {durs[len(durs) // 2]:.2f}s  peak torque median {peaks[len(peaks) // 2]:.0f}")
print("\n'otro' lane offset and lines:")
ot = [e for e in allrows if e["cls"] == "otro"]
print("  |off| median", sorted(abs(e["off"]) for e in ot)[len(ot) // 2] if ot else None,
      " car left:", sum(1 for e in ot if e["off"] > 0.1), " car right:", sum(1 for e in ot if e["off"] < -0.1),
      " one line weak (<0.5):", sum(1 for e in ot if min(e["pl"], e["pr"]) < 0.5))
print("\nposition corrections: lane centre offset at takeover (+ = car left of centre):")
pc = [e for e in allrows if e["cls"] == "corrección de posición en carril"]
print("  car left:", sum(1 for e in pc if e["off"] > 0), " car right:", sum(1 for e in pc if e["off"] < 0))
print("\nexamples per class:")
for c in cnt:
  for e in [x for x in allrows if x["cls"] == c][:4]:
    print(f"  {c[:22]:22} {e['route']} t+{e['t']:6.1f} {e['v']:3.0f} km/h angle {e['angle']:6.1f} des {e['des']:+.4f} act {e['act']:+.4f} "
          f"pinned {int(e['pinned'])} off {e['off']:+.2f} lines {e['pl']:.2f}/{e['pr']:.2f} blk {e['blk'] or '-'} against {int(e['against'])}")
