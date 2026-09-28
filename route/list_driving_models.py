"""Driving model catalogue on the device: big bundles in the regular and Chestnut caches, and the active bundle.

Reads the ModelManager params directly, no route needed. Big models need the Chestnut eGPU.

Usage: python list_driving_models.py
"""
import json
for f in ("ModelManager_ModelsCache", "ModelManager_ModelsCache_Chestnut"):
  c = json.load(open("/data/params/d/" + f))
  b = c["bundles"]
  big = [x for x in b if x.get("is_big")]
  print(f"== {f}: {len(b)} bundles, {len(big)} big")
  for x in sorted(big, key=lambda x: x["index"]):
    o = x.get("overrides", {})
    print(f"  {x['index']:3} {x['short_name']:10} {x['display_name']:50} gen {x['generation']} {x['environment']:11} "
          f"20hz={x['is_20hz']} folder={o.get('folder', '')} lat={o.get('lat', '')} long={o.get('long', '')} "
          f"build={x['build_time'][:10]} types={[m['type'] for m in x['models']]}")
a = json.load(open("/data/params/d/ModelManager_ActiveBundle"))
print("active:", a.get("displayName"), {k: v for k, v in a.items() if k.lower() in ("isbig", "is_big", "generation", "shortname")})
