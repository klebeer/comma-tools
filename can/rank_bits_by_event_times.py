"""Rank CAN bits by how well their edges line up with given event times.

Event times are seconds from the route start (e.g. beep times from find_beeps.py). A bit hits an
event when one of its edges falls between PRE s before and POST s after it. Bits are ranked by
hits minus the hits expected by chance if their edges were spread evenly over the route. Only
car bus (src 0) and camera frames forwarded to the car (src 128) are scanned. Ends with the
driving context at each event.

Usage: python rank_bits_by_event_times.py <route_id> <t1,t2,...>
"""
import collections
import os
import sys
import warnings

warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
PRE, POST = 1.5, 0.3
MAX_EDGES = 3000  # bits with more edges are dropped as too noisy to correlate


def main():
  prefix = sys.argv[1]
  events = [float(x) for x in sys.argv[2].split(",")]
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  t0 = None
  prev = {}
  edges = collections.defaultdict(list)
  overflow = set()
  state = collections.defaultdict(list)
  for s in segs:
    for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
      w = m.which()
      t = m.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      rel = t - t0
      if w == "carState":
        cs = m.carState
        state["blinker"].append((rel, ("L" if cs.leftBlinker else "") + ("R" if cs.rightBlinker else "")))
        state["steeringPressed"].append((rel, cs.steeringPressed))
        state["gear"].append((rel, str(cs.gearShifter)))
      if w != "can":
        continue
      for c in m.can:
        if c.src not in (0, 128):
          continue
        key = (c.src, c.address)
        val = int.from_bytes(c.dat, "big")
        n = len(c.dat) * 8
        p = prev.get(key)
        if p is not None and p != val:
          d = p ^ val
          while d:
            b = d & -d
            k = (key, n - b.bit_length())
            if k not in overflow:
              edges[k].append((rel, 1 if val & b else 0))
              if len(edges[k]) > MAX_EDGES:
                overflow.add(k)
                del edges[k]
            d ^= b
        prev[key] = val

  rows = []
  for k, e in edges.items():
    times = [t for t, _ in e]
    hits = [ev for ev in events if any(ev - PRE <= t <= ev + POST for t in times)]
    if len(hits) >= 3 and len(e) <= 400:
      rows.append((len(hits), len(e), k, hits))
  dur = max(t for e in edges.values() for t, _ in e) if edges else 1.0
  scored = []
  for hits, n, k, hl in rows:
    p_win = min(1.0, n * (PRE + POST) / dur)
    expected = p_win * len(events)
    scored.append((hits - expected, hits, n, k, hl, expected))
  scored.sort(key=lambda r: -r[0])
  rows = scored
  print(f"{prefix}: {len(events)} events, window -{PRE}s/+{POST}s, {len(edges)} bits tracked, {len(overflow)} too noisy")
  for lift, hits, n, ((src, addr), bit), hl, expected in rows[:25]:
    print(f"  src{src:3d} 0x{addr:03X} bit{bit:2d}  hits {hits:2d}/{len(events)}  expected by chance {expected:4.1f}  total edges {n:5d}  "
          f"missed {[ev for ev in events if ev not in hl]}")
  # The beep candidates: 0x4FA DBC bits 38/37 and 0x21C DBC bits 26/25/39, in this script's numbering.
  for (src, addr, bit) in ((0, 0x4FA, 33), (0, 0x4FA, 34), (128, 0x21C, 29), (128, 0x21C, 30), (128, 0x21C, 32)):
    k = ((src, addr), bit)
    if k in overflow:
      print(f"  check src{src} 0x{addr:03X} bit{bit}: too noisy"); continue
    e = edges.get(k, [])
    ts = [t for t, _ in e]
    hl = [ev for ev in events if any(ev - PRE <= t <= ev + POST for t in ts)]
    print(f"  check src{src} 0x{addr:03X} bit{bit}: total edges {len(e)}, hits {len(hl)}/{len(events)} {hl}")
  print("context at each event (blinker, steeringPressed, gear):")
  for ev in events:
    ctx = []
    for name in ("blinker", "steeringPressed", "gear"):
      s = [v for t, v in state[name] if ev - 1.0 <= t <= ev + 0.5]
      ctx.append(f"{name}={sorted(set(map(str, s)))}")
    print(f"  t+{ev:6.1f}s  " + "  ".join(ctx))


if __name__ == "__main__":
  main()
