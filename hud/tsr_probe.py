"""Temporary cluster discovery harness: steps CAM_TRAFFIC_SIGNS fields while parked. Not for the repo.

The trim has no traffic sign recognition. The first run found that the instrument cluster draws
the sign once byte 4 bits 1-0 (NEW_SIGNAL_4, SPEED_SIGN_CAM) are set; the windshield HUD showed
nothing. This run starts from that frame and sets one more bit per step in the bytes the first run
left alone (2, 3, 7 and byte 6 bits 6-5), each step with its own speed value so a sign that
appears on the HUD identifies its bit. Byte 5, FORWARD_COLLISION, is never touched.

Runs only when /data/tsr_probe.json exists, the car is in park at standstill, and openpilot is
steering (the panda only forwards openpilot's 0x35F then). Any other gear or motion aborts it and
the trigger is deleted when done or aborted. Needs the probe firmware, which accepts these fields
while the car is stopped. Progress goes to /data/tsr_probe.log with time.monotonic().

Not run directly: carcontroller_tsr_probe.patch calls TsrProbe.frame() for each camera 0x35F
frame and sends its result instead of the normal relay while it returns bytes.
"""
import os
import time

from opendbc.car import structs

TRIGGER = "/data/tsr_probe.json"
LOG = "/data/tsr_probe.log"
ON_S, GAP_S = 6.0, 3.0
GEAR_PARK = structs.CarState.GearShifter.park

# (speed shown, byte, bit) for each extra bit, over the frame the cluster already draws
EXTRA_BITS = [(b, i) for b in (2, 3, 7) for i in range(7, -1, -1)] + [(6, 6), (6, 5)]
STEPS = [(20 + 4 * k, byte, bit) for k, (byte, bit) in enumerate(EXTRA_BITS)]


def make_frame(cam_frame: bytes, step) -> bytes:
  speed, byte, bit = step
  dat = bytearray(cam_frame)
  dat[0] = (dat[0] & 0xe0) | ((speed >> 2) & 0x1f)
  dat[1] = (dat[1] & 0x0f) | ((speed & 0x3) << 6) | (2 << 4)
  dat[4] |= 0x03
  dat[byte] |= 1 << bit
  return bytes(dat)


class TsrProbe:
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

  def frame(self, CS, cam_frame: bytes) -> bytes | None:
    """The 0x35F to send in place of the relay, the camera's own frame in a gap, None when idle."""
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
    on = phase < ON_S
    if (i, on) != self.last_step:
      self._log(f"step {i + 1:2d} {'ON ' if on else 'gap'} {STEPS[i]}")
      self.last_step = (i, on)
    return make_frame(cam_frame, STEPS[i]) if on else cam_frame
