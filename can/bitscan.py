"""Per-bit activity of every CAN frame in a route, for diffing alert candidates between routes.

Usage on device:
  PYTHONPATH=/data/openpilot:/data/openpilot/openpilot /usr/local/venv/bin/python /data/bitscan.py <route_prefix> <out.json>
"""
import json
import os
import sys
import warnings

warnings.filterwarnings("ignore")

from openpilot.tools.lib.logreader import LogReader

BASE = "/data/media/0/realdata"
MAX_EDGES = 40


def main():
  prefix, out = sys.argv[1], sys.argv[2]
  segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))
  prev, frames, ones, flips, rises = {}, {}, {}, {}, {}
  t0, v, lat = None, 0.0, False
  for s in segs:
    p = os.path.join(BASE, s, "rlog.zst")
    if not os.path.exists(p):
      continue
    for m in LogReader(p):
      w = m.which()
      if w == "carState":
        v = m.carState.vEgo * 3.6
        continue
      if w == "carControl":
        lat = m.carControl.latActive
        continue
      if w != "can":
        continue
      t = m.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      for c in m.can:
        key = (c.src, c.address)
        val = int.from_bytes(c.dat, "big")
        n = len(c.dat) * 8
        if key not in frames:
          frames[key] = 0
          ones[key] = [0] * 64
          flips[key] = [0] * 64
          rises[key] = [[] for _ in range(64)]
        frames[key] += 1
        o = ones[key]
        x = val
        while x:
          b = x & -x
          o[n - b.bit_length()] += 1
          x ^= b
        if key in prev:
          d = prev[key] ^ val
          f, r = flips[key], rises[key]
          while d:
            b = d & -d
            i = n - b.bit_length()
            f[i] += 1
            if val & b and len(r[i]) < MAX_EDGES:
              r[i].append((round(t - t0, 1), round(v), int(lat)))
            d ^= b
        prev[key] = val

  res = []
  for key, nf in frames.items():
    for i in range(64):
      if flips[key][i] == 0 and ones[key][i] in (0, nf):
        continue
      res.append({"bus": key[0], "addr": key[1], "bit": i, "frames": nf, "flips": flips[key][i],
                  "duty": ones[key][i] / nf, "rises": rises[key][i]})
  with open(out, "w") as f:
    json.dump({"route": prefix, "duration": 0 if t0 is None else round(t - t0), "bits": res}, f)
  print(f"{prefix}: {len(frames)} (bus,addr) pairs, {len(res)} active bits")


if __name__ == "__main__":
  main()
