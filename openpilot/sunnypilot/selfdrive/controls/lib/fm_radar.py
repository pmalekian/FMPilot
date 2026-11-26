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

Lead confidence (Radar-matched / Camera-only Lead Confidence, 25–50%, default 50% = openpilot's fixed cutoff): attempt
radar matching above the lower cutoff and fall back to camera above the camera cutoff. From FrogPilot's "Lead Detection
Sensitivity" (LeadDetectionThreshold; FrogAi, FrogPilot branch, Oct 31, 2026, frogpilot/common/frogpilot_variables.py
and selfdrive/controls/radard.py, MIT), split into two cutoffs by FM Pilot.

Radar Lead Status (fmRadarLead, always logged): what radard did with the leads each frame, for the log, the Lead readout
and tools/fm/lead.py. Logging only: nothing here feeds back into radarState.

Ported to fm-dev-sync from fm-dev-nav-settle (ca08aac9) unchanged except: Lead Shadow Log dropped, and the old shared
FM_LeadDetectionThreshold is no longer read (the two confidence keys and their meanings are the same as there).
"""
import math
from dataclasses import dataclass

from openpilot.common.realtime import DT_MDL

PARAM_GUARD = "FM_RadarStationaryGuard"
PARAM_RADAR = "FM_RadarLeadConfidence"
PARAM_CAMERA = "FM_CameraLeadConfidence"
PARAM_SMOOTH = "FM_SmoothLeadHandoffs"

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
  if not math.isfinite(pct):
    return DEFAULT_THRESHOLD
  return min(max(pct / 100.0, THRESHOLD_LIMITS[0]), THRESHOLD_LIMITS[1])


@dataclass(frozen=True)
class FmLeadConfig:
  radar: float = DEFAULT_THRESHOLD
  camera: float = DEFAULT_THRESHOLD
  smooth: bool = False


class FmRadarSettings:
  """Poll once per second. With params=None replay can set the configuration explicitly."""

  def __init__(self, params=None):
    self.params = params
    self.guard = False
    self.config = FmLeadConfig()
    self.frame = -1
    if self.params is not None:
      self.read()

  @property
  def threshold(self):
    return min(self.config.radar, self.config.camera)

  def flag(self, key):
    try:
      return bool(self.params.get_bool(key))
    except Exception:
      return False

  def confidence(self, key):
    try:
      value = self.params.get(key)
    except Exception:
      return DEFAULT_THRESHOLD
    return threshold_from_percent(value)

  def read(self):
    self.guard = self.flag(PARAM_GUARD)
    self.config = FmLeadConfig(self.confidence(PARAM_RADAR), self.confidence(PARAM_CAMERA), self.flag(PARAM_SMOOTH))

  def update(self):
    self.frame += 1
    if self.params is not None and self.frame % int(PARAMS_PERIOD / DT_MDL) == 0:
      self.read()


# --- FM Pilot Radar Lead Status (fmRadarLead): what radard did with the leads, for the log and the Lead readout ---

SOURCE_NONE, SOURCE_RADAR, SOURCE_CAMERA = 0, 1, 2  # custom.FmRadarLead.Source


ADMIT_NONE, ADMIT_RADAR, ADMIT_CAMERA, ADMIT_LOW_SPEED, ADMIT_HOLD = range(5)
DIRECT, BLEND, HOLD = range(3)


class FmLeadTrace:
  """Filled by get_lead when given one: whether the ghost guard dropped a radar point this frame (and which), and
  whether the low-speed radar path replaced the lead. Logging only; get_lead's result doesn't depend on it."""
  __slots__ = ("ghost", "low_speed", "admission", "phase", "hysteresis")

  def __init__(self):
    self.ghost = None  # (dRel, yRel, vLead, trackId) of the radar point the guard dropped, or None
    self.low_speed = False
    self.admission = ADMIT_NONE
    self.phase = DIRECT
    self.hysteresis = False

  def decision(self, lead, probability):
    present = lead["present"]
    source = SOURCE_RADAR if lead.get("radar", False) else SOURCE_CAMERA if present else SOURCE_NONE
    return {
      "source": source,
      "confidence": float(probability),
      "radarTrackId": lead.get("radarTrackId", -1),
      "dRel": lead.get("dRel", 0.0),
      "vRel": lead.get("vRel", 0.0),
      "ghostDropped": self.ghost is not None,
      "ghostTrackId": int(self.ghost[3]) if self.ghost is not None else -1,
      "lowSpeedLead": self.low_speed,
      "admission": self.admission,
      "phase": self.phase,
      "hysteresis": self.hysteresis,
    }


EARLY_MATCH_PROB = 0.5  # "under 50%": openpilot's fixed cutoff


class FmRadarLeadStatus:
  """Builds fmRadarLead from leadOne and its trace each radard frame, and counts this drive's guard drops and early radar
  matches (radard runs once per drive, so both start at 0 each drive)."""

  def __init__(self):
    self.ghost_drops = 0
    self._ghost_prev = False
    self.early_radar_matches = 0
    self._early_match_prev = False

  def fill(self, msg, lead_one, trace: FmLeadTrace, settings: FmRadarSettings, live_prob: float | None = None) -> None:
    present = bool(lead_one.get('present', False))
    radar = present and bool(lead_one.get('radar', False))
    prob = float(lead_one.get('modelProb', 0.0)) if present else 0.0
    # Lead readout: the camera's confidence every frame (not just with a lead), whether this is the low-speed radar
    # path, and a count of radar matches made while the camera was under 50% (rising edge, like ghostDrops)
    msg.liveLeadProb = float(live_prob) if live_prob is not None else prob
    msg.lowSpeedLead = present and trace.low_speed
    early_match = radar and not trace.low_speed and prob < EARLY_MATCH_PROB
    if trace.phase != HOLD:
      if early_match and not self._early_match_prev:
        self.early_radar_matches += 1
      self._early_match_prev = early_match
    msg.earlyRadarMatches = self.early_radar_matches
    msg.source = SOURCE_RADAR if radar else (SOURCE_CAMERA if present else SOURCE_NONE)
    msg.leadProb = prob
    # a lead the camera is only threshold..50% sure of: openpilot's fixed 50% wouldn't have it yet
    msg.earlyLead = present and not trace.low_speed and prob <= DEFAULT_THRESHOLD
    ghost = trace.ghost is not None
    if ghost and not self._ghost_prev:
      self.ghost_drops += 1
    self._ghost_prev = ghost
    msg.ghostDropped = ghost
    if ghost:
      msg.ghostDRel, msg.ghostYRel, msg.ghostVLead, msg.ghostTrackId = (float(trace.ghost[0]), float(trace.ghost[1]),
                                                                       float(trace.ghost[2]), int(trace.ghost[3]))
    else:
      msg.ghostTrackId = -1
    msg.ghostDrops = self.ghost_drops
    msg.radarThreshold = settings.config.radar
    msg.cameraThreshold = settings.config.camera
    msg.smoothHandoffs = settings.config.smooth
    msg.guardEnabled = settings.guard
