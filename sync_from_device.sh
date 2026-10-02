#!/usr/bin/env bash
# Copies the device's route logs and shadow logs to this machine, under ~/.comma/media/0, exports
# the GPS track of each new drive to ~/.comma/tracks, has comma-nav rebuild the route bundle from
# them, and puts the new bundle on the device.
# Only new files transfer. Everything runs only while the car is off: nothing starts if the device
# is onroad or unreachable, and a copy stops as soon as either becomes true.
#
# Usage: ./sync_from_device.sh [device-ip]
set -u

DEVICE="${1:-192.168.1.35}"
DEST="$HOME/.comma/media/0"
SSH=(ssh -o ConnectTimeout=10 -o ServerAliveInterval=10 -o ServerAliveCountMax=3 -o HostKeyAlias="$DEVICE" -i "$HOME/.ssh/id_ed25519")

offroad() {
  [ "$("${SSH[@]}" "comma@$DEVICE" 'cat /data/params/d/IsOffroad' 2>/dev/null)" = "1" ]
}

copy() {
  local what="$1"; shift
  offroad || { echo "$what: device is onroad or unreachable, not copying"; return 1; }
  "${SSH[@]}" "comma@$DEVICE" "test -d /data/media/0/$what" || { echo "$what: nothing on the device yet"; return 0; }
  mkdir -p "$DEST/$what"
  rsync -a "$@" -e "${SSH[*]}" "comma@$DEVICE:/data/media/0/$what/" "$DEST/$what/" &
  local pid=$!
  while kill -0 "$pid" 2>/dev/null; do
    sleep 10
    if kill -0 "$pid" 2>/dev/null && ! offroad; then
      kill "$pid"
      echo "$what: device went onroad or dropped off the network, copy stopped"
      return 1
    fi
  done
  wait "$pid" || { echo "$what: rsync failed"; return 1; }
  echo "$what: $(find "$DEST/$what" -type f | wc -l | tr -d ' ') files"
}

copy realdata --prune-empty-dirs --include='*/' --include='rlog.zst' --exclude='*' || exit 1
for logs in curve_shadow lane_shadow nav_shadow mapd_log; do
  copy "$logs" || exit 1
done

HERE="$(cd "$(dirname "$0")" && pwd)"
OPENPILOT="$HOME/Projects/zoompilot-cx5"
PYTHONPATH="$OPENPILOT:$OPENPILOT/openpilot:$HOME/Projects/opendbc-cx5" "$HERE/.venv/bin/python" "$HERE/mapd/export_tracks.py" || exit 1

# From its own directory, where comma-nav keeps its map, its saved places and its saved trips.
NAV="$HOME/Projects/comma-nav"
if [ -x "$NAV/.venv/bin/comma-nav" ]; then
  (cd "$NAV" && .venv/bin/comma-nav bundle) || exit 1
  # Written under another name and renamed, so curve_shadow never reads half a file.
  offroad || { echo "bundle: device is onroad or unreachable, not sent"; exit 1; }
  "${SSH[@]}" "comma@$DEVICE" 'mkdir -p /data/nav' &&
    scp -q "${SSH[@]:1}" "$NAV/out/route_bundle.json" "comma@$DEVICE:/data/nav/route_bundle.json.tmp" &&
    "${SSH[@]}" "comma@$DEVICE" 'mv -f /data/nav/route_bundle.json.tmp /data/nav/route_bundle.json' &&
    echo "bundle: on the device, used from the next drive" || { echo "bundle: could not be sent"; exit 1; }
fi
