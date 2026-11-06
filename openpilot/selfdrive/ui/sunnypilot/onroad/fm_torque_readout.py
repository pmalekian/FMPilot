"""
FM Pilot torque readout: one line at the top center of the comma 4 road view.

  ▶ 457 / 481 / 530 ✓
  sent / limit at this speed / Max Torque in force this drive / EPAS taking it

- sent: carOutput.actuatorsOutput.torqueOutputCan, raw CAN units, not capped for display. The arrow is its direction
  (torque is positive to the left).
- limit now: carStateSP.fmSteerCap (speed curve, incl. AP's 95% cap past 90 degrees unless Full Torque Past 90° is on).
- Max Torque: carStateSP.fmMaxTorque, the low-speed limit the panda enforces this drive (385 without Panda Allows
  Higher Torque). A slider change shows from the next drive, as it applies.
- check: carStateSP.fmEpasToiActive (EPAS_SystemStatus ToiActive). It says the EPAS is taking the overlay, not how much
  it applied: the car reports no applied torque.

Colors: white; amber when sent is at 95% or more of the limit now; red with a cross when steering is on and the EPAS is
not taking it; grey "--" while steering is off. Always shown on a Rivian; nothing on other cars (no limit reported).
"""
from dataclasses import dataclass

AT_CAP_FRAC = 0.95


@dataclass(frozen=True)
class Readout:
  direction: int  # -1 left, +1 right, 0 none
  sent: str
  cap: str
  max_torque: str
  accepted: bool | None  # None while steering is off
  state: str  # 'normal' | 'at_cap' | 'refused' | 'off'


def make_readout(sent: float, steer_cap: int, max_torque: int, toi_active: bool, lat_active: bool) -> Readout | None:
  if max_torque <= 0:
    return None
  cap_txt = str(int(steer_cap)) if steer_cap > 0 else "--"
  if not lat_active:
    return Readout(0, "--", cap_txt, str(int(max_torque)), None, 'off')
  value = int(round(sent))
  direction = 0 if value == 0 else (-1 if value > 0 else 1)
  if not toi_active:
    state = 'refused'
  elif steer_cap > 0 and abs(value) >= AT_CAP_FRAC * steer_cap:
    state = 'at_cap'
  else:
    state = 'normal'
  return Readout(direction, str(abs(value)), cap_txt, str(int(max_torque)), toi_active, state)
