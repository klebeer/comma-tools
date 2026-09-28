#!/bin/bash
# Pause mapd_manager, replay a route's positions into mapd, and resume it on any exit.
# Usage: replay_route_into_mapd.sh <route_prefix> <step_s> <out.jsonl>
P=$(pgrep -f "openpilot.sunnypilot.mapd.mapd_manager" | head -1)
[ -n "$P" ] || { echo "mapd_manager not found"; exit 1; }
trap "kill -CONT $P; echo resumed mapd_manager $P" EXIT INT TERM HUP
kill -STOP $P && echo "paused mapd_manager $P"
cd /data && PYTHONPATH=/data/openpilot:/data/openpilot/openpilot timeout 1500 /usr/local/venv/bin/python /data/replay_route_into_mapd.py "$@"
