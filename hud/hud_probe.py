"""Temporary HUD discovery harness: steps CAM_LANEINFO fields while parked. Not for the repo.

Runs only when /data/hud_probe.json exists and the car is in park at standstill. Any other gear or
motion aborts it. Deletes the trigger when done or aborted. Each step is logged with
time.monotonic() so it joins the rlog (logMonoTime) and a phone video of the HUD.

Not run directly: carcontroller_probe.patch imports it into opendbc's Mazda carcontroller, which
calls HudProbe.overrides(CS) every frame and, while it returns a dict, sends make_msg() instead of
the normal CAM_LANEINFO alert command. Each step holds its fields for ON_S, then GAP_S of
baseline. Progress goes to /data/hud_probe.log.
"""
import os
import time

from opendbc.car import structs

TRIGGER = "/data/hud_probe.json"
LOG = "/data/hud_probe.log"
ON_S, GAP_S = 6.0, 3.0
GEAR_PARK = structs.CarState.GearShifter.park

COPY = ["LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT2", "BIT3", "NO_ERR_BIT", "ERR_BIT", "S1", "S1_HBEAM"]
LINES = {"LANE_LINES": 2, "LINE_VISIBLE": 1, "LINE_NOT_VISIBLE": 0}

STEPS = (
  [(f"LANE_LINES={v}", {"LANE_LINES": v, "LINE_VISIBLE": 1, "LINE_NOT_VISIBLE": 0}) for v in range(5)]
  + [("LINE_NOT_VISIBLE", {"LANE_LINES": 2, "LINE_VISIBLE": 0, "LINE_NOT_VISIBLE": 1})]
  + [(f"TJA={v}", {**LINES, "TJA": v}) for v in range(8)]
  + [(f"TJA=4 TJA_TRANSITION={v}", {**LINES, "TJA": 4, "TJA_TRANSITION": v}) for v in (1, 2, 3)]
  + [("LDW_WARN_LL", {**LINES, "LDW_WARN_LL": 1}), ("LDW_WARN_RL", {**LINES, "LDW_WARN_RL": 1})]
  + [("HANDS_ON_STEER_WARN", {**LINES, "HANDS_ON_STEER_WARN": 1}),
     ("HANDS_ON_STEER_WARN_2", {**LINES, "HANDS_ON_STEER_WARN_2": 1})]
  + [(f"HANDS_WARN_3_BITS={v}", {**LINES, "HANDS_WARN_3_BITS": v}) for v in (1, 2, 4, 7)]
  + [("S1=1", {**LINES, "S1": 1}), ("S1_HBEAM=1", {**LINES, "S1_HBEAM": 1})]
  + [("ERR_BIT=1", {**LINES, "ERR_BIT": 1, "NO_ERR_BIT": 0})]
)


class HudProbe:
  def __init__(self):
    self.start = None
    self.done = False
    self.last_step = None
    self.next_check = 0.0
    self.armed = False

  def _log(self, text):
    with open(LOG, "a") as f:
      f.write(f"{time.monotonic():.3f} {text}\n")

  def _finish(self, why):
    self.done = True
    self._log(why)
    try:
      os.remove(TRIGGER)
    except OSError:
      pass

  def overrides(self, CS) -> dict | None:
    """Field overrides for this frame, {} for a baseline gap, None when not probing."""
    if self.done:
      return None
    now = time.monotonic()
    if not self.armed:
      if now < self.next_check:
        return None
      self.next_check = now + 1.0
      self.armed = os.path.exists(TRIGGER)
      if not self.armed:
        return None
    parked = CS.out.gearShifter == GEAR_PARK and CS.out.standstill
    if not parked:
      if self.start is not None:
        self._finish("aborted: left park")
      return None
    if self.start is None:
      self.start = now
      self._log(f"start {len(STEPS)} steps, {ON_S}s on / {GAP_S}s baseline")
    i, phase = divmod(now - self.start, ON_S + GAP_S)
    i = int(i)
    if i >= len(STEPS):
      self._finish("done")
      return None
    name, values = STEPS[i]
    on = phase < ON_S
    step = (i, on)
    if step != self.last_step:
      self._log(f"step {i + 1:2d} {'ON ' if on else 'gap'} {name}")
      self.last_step = step
    return values if on else {}


def make_msg(packer, cam_msg, values):
  msg = {s: cam_msg[s] for s in COPY}
  msg.update(values)
  return packer.make_can_msg("CAM_LANEINFO", 0, msg)
