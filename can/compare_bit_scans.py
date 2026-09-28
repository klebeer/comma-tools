"""Bits that were constant or absent before the panda flash and toggle after it, named from the DBC.

Reads two scan_bit_activity.py outputs from <dir>, fixed as bits_17.json (before) and
bits_1d.json (after). Keeps car-side bits (bus < 128) that flip 1-600 times and are high under
half the time. Runs on the Mac.

Usage: python compare_bit_scans.py <dir> <mazda_2017.dbc>
"""
import json, re, sys
D = sys.argv[1]
dbc = open(sys.argv[2]).read()
msgs, sigs = {}, {}
cur = None
for line in dbc.splitlines():
  m = re.match(r"BO_ (\d+) (\w+):", line)
  if m:
    cur = int(m.group(1)); msgs[cur] = m.group(2); continue
  m = re.match(r"\s+SG_ (\w+) : (\d+)\|(\d+)@0", line)
  if m and cur is not None:
    name, sb, ln = m.group(1), int(m.group(2)), int(m.group(3))
    b = sb
    # Motorola (big-endian) signal: walk from the start bit and wrap to the next byte's MSB.
    for _ in range(ln):
      byte, bit = divmod(b, 8)
      sigs[(cur, byte * 8 + (7 - bit))] = name
      b = b - 1 if bit > 0 else (byte + 1) * 8 + 7

def load(f):
  j = json.load(open(f"{D}/{f}"))
  return j["duration"], {(x["bus"], x["addr"], x["bit"]): x for x in j["bits"]}

d17, pre = load("bits_17.json")
d1d, post = load("bits_1d.json")
print(f"pre 17: {d17}s   post 1d: {d1d}s")
rows = []
for k, x in post.items():
  bus, addr, bit = k
  if bus >= 128:
    continue
  if not (1 <= x["flips"] <= 600 and x["duty"] < 0.5):
    continue
  p = pre.get(k)
  pf = p["flips"] if p else None
  if p and p["flips"] > 0:
    continue
  rows.append((bus, addr, bit, x, pf))
rows.sort(key=lambda r: (r[0], r[1], r[2]))
for bus, addr, bit, x, pf in rows:
  name = f"{msgs.get(addr, '?')}.{sigs.get((addr, bit), '?')}"
  r = x["rises"][:6]
  print(f"bus{bus} 0x{addr:03X} bit{bit:2d} {name:38} flips {x['flips']:4d} duty {100*x['duty']:5.2f}%  pre {'absent' if pf is None else 'const'}  rises {r}")
