"""
FM Pilot: radar lead settings for radard.

Stationary Radar Ghost Guard (FM_RadarStationaryGuard, off by default). openpilot pairs the camera's lead car with the
radar point that best fits it, then keeps the pairing if the point is within 5 m (or 25%) of the camera's distance and
within 10 m/s of its speed. At ramp and curve speeds a guardrail or barrier point (0 m/s) passes both checks whenever the
real car isn't a radar point, and the planner brakes for a "stopped car" (Nov 5: 00000038--19 on-ramp, -1.8 to -3.5
m/s^2; also --13 and 00000039--47). With the guard on, a radar point standing still is never paired with a camera lead
that is clearly moving; radard then uses the camera's own estimate of that car, as it does whenever no point fits.
The guard stays out of the way when the camera sees the lead braking hard, so a car stopping suddenly is still taken
from the radar, and it never touches the low-speed path (radar points straight ahead below 4 m/s ego speed).
Thresholds from the Nov 5 logs: phantom camera leads were 4.4 m/s and up (7.8 typical) with camera accel -0.56 to
+0.32; real nearly stopped cars read up to 3.2 m/s on the camera (one 8.9 mph blip at 55 m), so 5 m/s leaves a margin.

Lead Detection Sensitivity (FM_LeadDetectionThreshold, percent; 50 = openpilot's fixed 0.5). How sure the camera has to
be that a car is ahead before it counts as a lead and can be paired with the radar. Lower values pick up leads sooner
and farther away, so the lead-following planner gets radar distance and speed earlier; they also give more chances to
follow something that isn't a car, so use it with the guard above. From FrogPilot's "Lead Detection Sensitivity"
(LeadDetectionThreshold; FrogAi, FrogPilot branch, frogpilot/common/frogpilot_variables.py and
selfdrive/controls/radard.py, MIT). FrogPilot allows 25-50% with a 35% default; FM Pilot keeps 50% as the default so
nothing changes until it is lowered.
"""
from openpilot.common.realtime import DT_MDL

PARAM_GUARD = "FM_RadarStationaryGuard"
PARAM_THRESHOLD = "FM_LeadDetectionThreshold"

GHOST_TRACK_MAX_SPEED = 1.0   # m/s: the radar point is standing still
GHOST_VISION_MIN_SPEED = 5.0  # m/s (11 mph): the camera's lead is clearly moving
GHOST_VISION_BRAKING = -1.5   # m/s^2: camera lead braking harder than this keeps the radar point

DEFAULT_THRESHOLD = 0.5
THRESHOLD_LIMITS = (0.25, 0.5)  # FrogPilot's range

PARAMS_PERIOD = 1.0  # s


def stationary_ghost(track_v_lead: float, vision_v: float, vision_a: float) -> bool:
  """True when a standing-still radar point is being paired with a camera lead that is clearly moving and not braking
  hard: they can't be the same object."""
  return (abs(track_v_lead) < GHOST_TRACK_MAX_SPEED and vision_v > GHOST_VISION_MIN_SPEED and
          vision_a > GHOST_VISION_BRAKING)


def threshold_from_percent(value) -> float:
  """Setting in percent -> probability, clamped to 25-50%; anything unreadable is openpilot's 0.5."""
  try:
    pct = float(value)
  except (TypeError, ValueError):
    return DEFAULT_THRESHOLD
  if pct != pct:  # NaN
    return DEFAULT_THRESHOLD
  return min(max(pct / 100.0, THRESHOLD_LIMITS[0]), THRESHOLD_LIMITS[1])


class FmRadarSettings:
  """Reads the settings once per second. With params=None (tests, replay) everything stays at openpilot's behavior."""

  def __init__(self, params=None):
    self.params = params
    self.guard = False
    self.threshold = DEFAULT_THRESHOLD
    self.frame = -1
    if self.params is not None:
      self.read()

  def read(self):
    try:
      self.guard = bool(self.params.get_bool(PARAM_GUARD))
    except Exception:
      self.guard = False
    try:
      self.threshold = threshold_from_percent(self.params.get(PARAM_THRESHOLD, return_default=True))
    except Exception:
      self.threshold = DEFAULT_THRESHOLD

  def update(self):
    self.frame += 1
    if self.params is not None and self.frame % int(PARAMS_PERIOD / DT_MDL) == 0:
      self.read()
