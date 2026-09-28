#!/bin/bash
for r in 032e2fb0c8 c540fbef83 eb2f9b1e95; do
  /data/mapd_replay.sh $r 4 /data/mapd_$r.jsonl
done
echo ALL_DONE
