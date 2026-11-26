"""
FM Pilot torque readout: one line at the top center of the comma 4 road view.

  ▶ 457 / 481 / 530 ✓
  sent / limit at this speed / Max Torque in force this drive / EPAS taking it

- sent: carOutput.actuatorsOutput.torqueOutputCan, raw CAN units, not capped for display. The arrow is its direction
  (torque is positive to the left).
- limit now: fmCarState.steerCap (speed curve, incl. AP's 95% cap past 90 degrees when Max Torque is 385).
- Max Torque: fmCarState.maxTorque, the low-speed limit the panda enforces this drive. A slider change shows from the
  next drive, as it applies.
- check: fmCarState.epasToiActive (EPAS_SystemStatus ToiActive). It says the EPAS is taking the overlay, not how much
  it applied: the car reports no applied torque.

Colors: white; amber when sent is at 95% or more of the limit now; red with a cross when steering is on and the EPAS is
not taking it; grey "--" while steering is off. Shown on a Rivian (nothing on other cars: no limit reported).

FM_TopReadout picks what the line shows: 0 Off (default), 1 Torque (above), 2 CPU with the average of the other cores,
3 CPU with the busiest other core:

  4: 99% / A: 68% / 68°C      4: 99% / B: 74% / 68°C

Core 4 is where card, controlsd and selfdrived run; "A" is the average of the other cores, "B" the busiest of them, and
the temperature is the hottest CPU sensor (deviceState, 2 Hz). Amber when core 4 is at 95% or more.

4 Lead: what radard is doing with the lead (fmRadarLead, 20 Hz), no labels:

  18% – E0 G0     38% C E0 G0     40% R E1 G0     87% R E1 G0     74% X E1 G1

- number: the camera's live lead confidence (fmRadarLead.liveLeadProb: modelV2.leadsV3[0].prob through radard's own
  asymmetric filter, so it is the value radard compared with Lead Detection Sensitivity). Gray: no lead counted (below
  the cutoff), and the low-speed radar path; cyan: camera-only lead under 50%; purple: radar-matched lead under 50%;
  white: any lead at 50% or more.
- letter: R radar-matched, C camera-only, X Ignore Stationary Radar Ghosts dropped a standing-still point (flashes red
  for about 2 s), – no lead.
- E: radar matches made this drive while the camera was under 50% sure (counted in radard, once per run).
- G: the guard's ghost drops this drive (fmRadarLead.ghostDrops).
"""
import math
from dataclasses import dataclass

AT_CAP_FRAC = 0.95
PARAM_TOP_READOUT = "FM_TopReadout"
READOUT_OFF, READOUT_TORQUE, READOUT_CPU_AVG, READOUT_CPU_BUSIEST, READOUT_LEAD = 0, 1, 2, 3, 4
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
  if max_torque <= 0 or (lat_active and not math.isfinite(sent)):
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
  except (TypeError, ValueError, OverflowError):
    return READOUT_OFF
  return mode if mode in (READOUT_OFF, READOUT_TORQUE, READOUT_CPU_AVG, READOUT_CPU_BUSIEST, READOUT_LEAD) else READOUT_OFF


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


# --- Lead (FM_TopReadout 4) ---

LEAD_SOURCE_NONE, LEAD_SOURCE_RADAR, LEAD_SOURCE_CAMERA = 0, 1, 2  # custom.FmRadarLead.Source
LEAD_SURE_PROB = 0.5     # white from 50%
GHOST_FLASH_S = 2.0      # how long X stays after the guard's last drop
GHOST_BLINK_S = 0.25     # X on / dim for this long each


@dataclass(frozen=True)
class LeadReadout:
  prob: str        # "40%"
  prob_color: str  # 'gray' | 'cyan' | 'purple' | 'white'
  letter: str      # 'R' | 'C' | 'X' | '–'
  letter_color: str  # 'white' | 'gray' | 'red' | 'red_dim' (X between flashes)
  counts: str      # "E1 G0"

  @property
  def text(self) -> str:
    return f"{self.prob} {self.letter} {self.counts}"


def lead_prob_color(prob: float, source: int, low_speed: bool) -> str:
  if source == LEAD_SOURCE_NONE or low_speed:
    return 'gray'
  if prob >= LEAD_SURE_PROB:
    return 'white'
  if source == LEAD_SOURCE_RADAR:
    return 'purple'
  if source == LEAD_SOURCE_CAMERA:
    return 'cyan'
  return 'gray'


def lead_prob_text(prob: float) -> str:
  if not math.isfinite(prob):
    return "--%"
  # floored, so a lead shown under 50% is never labelled "50%" (white starts at exactly 50%)
  return f"{math.floor(min(max(prob, 0.0), 1.0) * 100 + 1e-6)}%"


class LeadReadoutState:
  """Builds the Lead readout from fmRadarLead frame by frame; keeps the X flash going ~2 s after the guard's last drop,
  including a drop the UI missed between frames (ghostDrops went up). Pure logic, no drawing (tested on its own)."""

  def __init__(self):
    self.ghost_until = -math.inf
    self._ghost_drops: int | None = None

  def update(self, now: float, live_prob: float, source: int, low_speed: bool, ghost_dropped: bool, ghost_drops: int,
             early_matches: int) -> LeadReadout:
    new_drop = self._ghost_drops is not None and ghost_drops > self._ghost_drops
    self._ghost_drops = ghost_drops  # a lower count is a new drive: no flash
    if ghost_dropped or new_drop:
      self.ghost_until = now + GHOST_FLASH_S

    if now < self.ghost_until:
      letter = 'X'
      since = now - (self.ghost_until - GHOST_FLASH_S)  # time since the last drop
      letter_color = 'red' if int(since / GHOST_BLINK_S) % 2 == 0 else 'red_dim'
    elif source == LEAD_SOURCE_RADAR:
      letter, letter_color = 'R', 'white'
    elif source == LEAD_SOURCE_CAMERA:
      letter, letter_color = 'C', 'white'
    else:
      letter, letter_color = '–', 'gray'
    return LeadReadout(lead_prob_text(live_prob), lead_prob_color(live_prob, source, low_speed), letter, letter_color,
                       f"E{int(early_matches)} G{int(ghost_drops)}")
