"""Feed a logged route through the patched CarInterface capture and the relay builder."""
import os, sys, warnings
warnings.filterwarnings("ignore")
from openpilot.tools.lib.logreader import LogReader
from opendbc.car.can_definitions import CanData
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from opendbc.car import gen_empty_fingerprint
print("opendbc from", mazdacan.__file__)
CP = CarInterface.get_params(CAR.MAZDA_CX5_2022_NON_MRCC, gen_empty_fingerprint(), [], alpha_long=False, is_release=False, docs=False)
CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022_NON_MRCC, gen_empty_fingerprint(), [], alpha_long=False, is_release_sp=False, docs=False)
ci = CarInterface(CP, CP_SP)
BASE = "/data/media/0/realdata"; prefix = sys.argv[1]
segs = sorted((d for d in os.listdir(BASE) if "--" in d and d.split("--")[1] == prefix), key=lambda n: int(n.rsplit("--", 1)[1]))[:3]
cam = relays = mismatch = chime_in = chime_out = 0; last_seen = 0
for s in segs:
  for m in LogReader(os.path.join(BASE, s, "rlog.zst")):
    if m.which() != "can": continue
    pk = [(m.logMonoTime, [CanData(f.address, bytes(f.dat), f.src) for f in m.can])]
    cam_frames = [f for f in pk[0][1] if f.src == 2 and f.address == 0x21C]
    ci.update(pk)
    cam += len(cam_frames)
    if ci.CS.cam_crz_ctrl_frames != last_seen:
      last_seen = ci.CS.cam_crz_ctrl_frames
      src = ci.CS.cam_crz_ctrl
      out = mazdacan.create_crz_ctrl_relay(src)
      relays += 1
      chime_in += (src[4] >> 7) & 1
      chime_out += (out.dat[4] >> 7) & 1
      exp = bytearray(src); exp[4] &= 0x7F
      mismatch += out.dat != bytes(exp) or out.address != 0x21C or out.src != 0 or src != cam_frames[-1].dat
print(f"segments {len(segs)}: camera frames {cam}, captured {ci.CS.cam_crz_ctrl_frames}, relays built {relays}, "
      f"with chime in {chime_in}, chime out {chime_out}, mismatches {mismatch}")
