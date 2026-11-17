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
not taking it; grey "--" while steering is off. Shown on a Rivian (nothing on other cars: no limit reported).

FM_TopReadout picks what the line shows: 0 Off, 1 Torque (above, default), 2 CPU with the average of the other cores,
3 CPU with the busiest other core:

  4: 99% / A: 68% / 68°C      4: 99% / B: 74% / 68°C

Core 4 is where card, controlsd and selfdrived run; "A" is the average of the other cores, "B" the busiest of them, and
the temperature is the hottest CPU sensor (deviceState, 2 Hz). Amber when core 4 is at 95% or more.
"""
from dataclasses import dataclass

AT_CAP_FRAC = 0.95
READOUT_OFF, READOUT_TORQUE, READOUT_CPU_AVG, READOUT_CPU_BUSIEST = 0, 1, 2, 3
CONTROL_CORE = 4
CORE_BUSY = 95
TURN_NOTICE_TYPES = ("laneTurnLeft/warning", "laneTurnRight/warning")


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


def parse_readout_mode(value) -> int:
  try:
    mode = int(value)
  except (TypeError, ValueError):
    return READOUT_TORQUE
  return mode if mode in (READOUT_OFF, READOUT_TORQUE, READOUT_CPU_AVG, READOUT_CPU_BUSIEST) else READOUT_TORQUE


def make_cpu_readout(mode: int, cpu_usage, cpu_temps, max_temp: float) -> tuple[str, bool] | None:
  """(text, core 4 busy) for the CPU modes; None when there is nothing to show."""
  usage = [int(u) for u in cpu_usage]
  if len(usage) <= CONTROL_CORE:
    return None
  others = usage[:CONTROL_CORE] + usage[CONTROL_CORE + 1:]
  if mode == READOUT_CPU_AVG:
    other = f"A: {round(sum(others) / len(others))}%"
  else:
    other = f"B: {max(others)}%"
  temps = [t for t in cpu_temps if t > 0]
  temp = max(temps) if temps else max_temp
  temp_txt = f"{round(temp)}°C" if temp > 0 else "--°C"
  core = usage[CONTROL_CORE]
  return f"{CONTROL_CORE}: {core}% / {other} / {temp_txt}", core >= CORE_BUSY
