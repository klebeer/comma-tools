# comma-tools

Analysis tools for a comma four running zoompilot on a Mazda CX-5 2025 without MRCC
(`MAZDA_CX5_2022_NON_MRCC`). Every script reads the device's own route logs; none of them change
how the car drives. The one exception is `hud/`, a parked-only test harness that is not
installed by default.

The car code lives in `klebeer/zoompilot` (branch `cx5-2025-non-mrcc`) and `klebeer/opendbc`
(branch `cx5-non-mrcc-crz-ctrl-relay`).

## Running on the device

```bash
scp -i ~/.ssh/id_ed25519 <script>.py comma@<device-ip>:/data/
ssh -i ~/.ssh/id_ed25519 comma@<device-ip> \
  'cd /data && PYTHONPATH=/data:/data/openpilot:/data/openpilot/openpilot /usr/local/venv/bin/python <script>.py <route_id> ...'
```

`<route_id>` is the part after the counter, e.g. `9de3eaeb9b` for `00000024--9de3eaeb9b`.
Routes live in `/data/media/0/realdata`; archived rlogs in `/data/media/0/archive`; live mapd
records in `/data/media/0/mapd_log`. Long runs: start them with `nohup ... </dev/null &` and poll
with `ps aux | grep -c "[s]cript.py"` (a bare `pgrep -f` inside `bash -c` matches itself).
There is no pytest on the device.

## Scripts

### route/ — one-drive reports

| Script | Measures |
|---|---|
| `post_drive_check.py` | card and panda state, dead processes, the CRZ_CTRL relay on the wire, events |
| `drive_report.py` | tracking error by speed band, learned torque params, sounds, CAN warning bits |
| `lane_center_and_confidence.py` | lane centre offset and lane line confidence |
| `last_seconds.py` | the last N seconds in 10 s blocks: speed, lines, edges, camera intent, road name |
| `curve_timeline.py` | timeline of a window: desired vs actual curvature, torque, lane lines |
| `classify_takeovers.py` | classifies every driver takeover by context (lane change, tight turn, light touch...) |
| `stock_fcw_episodes.py` | stock collision warning (`stockFcw`) episodes |
| `export_gps_track.py` | GPS track to CSV with speed, accuracy and mapd road names |
| `list_driving_models.py` | driving model catalogue on the device (big vs regular) |

### lateral/ — steering precision

| Script | Measures |
|---|---|
| `curve_tracking_by_speed.py` | achieved/requested curvature by speed and curve tightness, and whether torque was capped |
| `curve_error_causes.py` | under-tracking in curves split by cause: EPS ceiling, slew limit, EPS delivery, controller room |
| `controller_terms_gentle_curves.py`, `controller_terms_by_turn_direction.py` | torque controller terms in gentle curves; tracking vs roll compensation |
| `saturation_check_replay.py` | replays the lateral saturation check under different rules |
| `steering_delay_offline.py` | runs lagd's own estimator offline with lower speed floors |
| `model_vs_stock_camera.py` | model lane position vs the stock camera's steering intent |
| `position_between_road_edges.py`, `wide_street_timeline.py` | position between the road edges on wide streets |
| `missing_right_line.py` | how often the right line is missing and the room to the curb |

### can/ — signal discovery

| Script | Measures |
|---|---|
| `scan_bit_activity.py` | per-bit activity of every CAN frame in a route, to JSON |
| `compare_bit_scans.py` | diffs two `scan_bit_activity.py` outputs, named with the DBC (runs on the Mac) |
| `bit_edges_in_window.py`, `bit_edges_around_beep.py` | bit edges inside a time window, with driving state |
| `rank_bits_by_event_times.py` | ranks bits by how well their edges line up with given event times |
| `crz_ctrl_bit39_vs_beeps.py` | CRZ_CTRL (0x21C) bit 39 intervals vs beep times |
| `msg_4fa_timeline.py` | 0x4FA field timeline with lane context |
| `eps_torque_delivery.py` | commanded torque vs EPS motor torque and LKAS_BLOCK |
| `crz_ctrl_rate_and_gaps.py` | camera CRZ_CTRL rate and largest gaps |

### audio/

| Script | Measures |
|---|---|
| `find_beeps.py` | route microphone audio to WAV and tonal events (needs `RecordAudio` on) |

### mapd/ — map data

| Script | Measures |
|---|---|
| `replay_route_into_mapd.sh`, `replay_routes_into_mapd.sh` | pause `mapd_manager`, replay a route's positions into mapd, resume |
| `replay_route_into_mapd.py` | the replay itself, one JSON line per position |
| `probe_one_position.py` | hands mapd one logged position and prints what it publishes |
| `map_vs_driven_curvature.py` ... `fixed_vs_learned_limit.py` | map curvature ahead vs what the car drove; fixed vs learned limits |
| `map_ahead_of_curves.py` | what mapd reported ahead of specific curves (live log or replay) |
| `evaluate_curve_warning.py` | judges the shadow curve warnings: real (torque cap, takeover, runs wide) or false, and misses |

`map_warning_false_alarms.py` and `fixed_vs_learned_limit.py` import `map_curve_warning_eval.py`: copy all three to `/data`.

### relay/

| Script | Measures |
|---|---|
| `replay_crz_ctrl_relay.py` | feeds a logged route through the CRZ_CTRL capture and relay builder |

### hud/ — parked HUD discovery (not installed)

`hud_probe.py` steps CAM_LANEINFO fields (lane lines, TJA, LDW, hands-on) while the car is in park,
and `carcontroller_probe.patch` hooks it into opendbc's Mazda carcontroller. It runs only when
`/data/hud_probe.json` exists, the car is in park at standstill and lateral is armed with ON;
leaving park aborts it and it deletes the trigger when done. Remove the patch after the test.

## Findings these tools produced

- The in-drive chime was camera CRZ_CTRL bit 39 relayed to the cluster (10/10 beeps matched).
- Gentle curves under 40 km/h ran wide with torque to spare; tracking fell as roll
  compensation grew (0.92 to 0.74). Tight curves at 13-21 km/h hit the torque cap with no alert.
- 43% of takeovers were light resting touches (< 0.5 s, torque < 40).
- With the right lane line missing (17.5% of town driving), the car sits off the left line and
  leaves room to the curb unused.
- Auto Hold: button 0x203 bit 31, armed 0x079 bit 25, hold active 0x228 BRAKE_HOLD.
