"""Exports each drive's GPS track for tools that do not read rlogs, such as comma-nav.

One file per drive in TRACKS_DIR, named <counter>--<route_id>.json so the names sort oldest first:
{"segments": N, "fixes": [[latitude, longitude, heading], ...]}, the fixes turn_memory uses (the car
moving at MIN_KPH or more). A drive is exported again when it has gained segments since.

Usage: python mapd/export_tracks.py [<route_id> ...]     (no arguments: every drive on disk)
"""
import json, os, sys
from turn_memory import BASE, track

TRACKS_DIR = os.path.expanduser("~/.comma/tracks")


def drives():
  """{route_id: (counter, segments on disk)}, oldest first."""
  found = {}
  for d in sorted(os.listdir(BASE)):
    parts = d.split("--")
    if len(parts) == 3 and os.path.exists(os.path.join(BASE, d, "rlog.zst")):
      counter, n = found.get(parts[1], (parts[0], 0))
      found[parts[1]] = (counter, n + 1)
  return found


def name(prefix, found=None):
  return f"{(found or drives())[prefix][0]}--{prefix}"


def load(prefix, found=None):
  """The drive's fixes, read from its export when that is current, else from the rlogs and exported."""
  found = found or drives()
  path = os.path.join(TRACKS_DIR, name(prefix, found) + ".json")
  try:
    with open(path) as f:
      saved = json.load(f)
    if saved["segments"] == found[prefix][1]:
      return [tuple(p) for p in saved["fixes"]]
  except (OSError, ValueError, KeyError):
    pass
  fixes = track(prefix)
  os.makedirs(TRACKS_DIR, exist_ok=True)
  with open(path + ".tmp", "w") as f:
    json.dump({"segments": found[prefix][1], "fixes": fixes}, f)
  os.replace(path + ".tmp", path)
  return fixes


def main():
  found = drives()
  before = set(os.listdir(TRACKS_DIR)) if os.path.isdir(TRACKS_DIR) else set()
  for prefix in sys.argv[1:] or found:
    load(prefix, found)
  now = set(os.listdir(TRACKS_DIR))
  print(f"{TRACKS_DIR}: {len(now)} tracks, {len(now - before)} new")


if __name__ == "__main__":
  main()
