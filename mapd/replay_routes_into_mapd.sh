#!/bin/bash
# Replay the listed routes into mapd every 4 s, one /data/mapd_<route>.jsonl each.
# Usage: replay_routes_into_mapd.sh
for r in 032e2fb0c8 c540fbef83 eb2f9b1e95; do
  /data/replay_route_into_mapd.sh $r 4 /data/mapd_$r.jsonl
done
echo ALL_DONE
