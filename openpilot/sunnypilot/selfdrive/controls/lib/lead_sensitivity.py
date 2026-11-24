"""
Custom Lead Sensitivity: settings for when radard starts following the camera's lead car.

Stock radard follows the driving model's lead car, and tries to pair it with a radar point, only once the model is more
than 50% sure of it. With Custom Lead Sensitivity on, two settings replace that fixed cutoff:

- Camera-only Lead Confidence: how sure the model must be to follow a car that no radar point confirms.
- Radar-matched Lead Confidence: how sure the model must be before radard tries to pair the car with a radar point.

Radar pairing starts at the lower of the two, so a car the radar confirms can be followed sooner than one it doesn't.

Smooth Lead Handoffs keeps one car steady as the lead. It predicts where the car should be each frame and treats a new
reading as the same car when it is within 1.5 m sideways, 4 m/s in speed and 4-10 m in distance. For that car:

- Release margin: it stays the lead until the model's confidence falls 5 points below the cutoff that admitted it.
- Hold: if a radar-backed lead drops out while the camera still sees the same car on the path, the prediction is kept
  for up to 0.5 s.
- Blend: when the source changes (radar to camera, or a new radar point for the same car), or after a hold, good news
  (farther, faster, braking less) is eased in over 0.75 s. Bad news always applies at once.
- Radar pairing stays on the point already being followed while it still fits the camera's car.

Ignore Stationary Radar Ghosts stops radard pairing a clearly moving camera car with a radar point that is standing
still. radard keeps a pairing whenever the point is within 5 m (or 25%) of the camera's distance and 10 m/s of its speed,
so on ramps and curves a guardrail or barrier point (0 m/s) passes whenever the real car isn't a radar point, and the
planner brakes for a "stopped car". With the guard on, a point moving under 1 m/s is not paired with a camera car moving
over 5 m/s (11 mph) unless the camera sees that car braking harder than -1.5 m/s^2; radard then follows the camera's own
estimate, if it clears the camera-only cutoff. The low-speed radar override is untouched.

With Custom Lead Sensitivity off, radard runs its stock code and these settings are ignored. Everything is off, and
both settings are at 50%, by default.

The adjustable confidence cutoff is adapted from FrogPilot's Lead Detection Sensitivity (FrogAi, MIT).

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
from dataclasses import dataclass

from openpilot.common.realtime import DT_MDL

PARAM_ENABLED = "CustomLeadSensitivity"
PARAM_CAMERA = "CameraLeadConfidence"
PARAM_RADAR = "RadarLeadConfidence"
PARAM_SMOOTH = "SmoothLeadHandoffs"
PARAM_GHOSTS = "IgnoreStationaryRadarGhosts"

DEFAULT_CONFIDENCE = 0.5  # stock radard's cutoff
CONFIDENCE_LIMITS = (0.25, 0.5)
PARAMS_PERIOD = 1.0  # s

# Smooth Lead Handoffs
RELEASE_MARGIN = 0.05
HOLD_FRAMES = round(0.5 / DT_MDL)
BLEND_FRAMES = round(0.75 / DT_MDL)
SAME_CAR_LATERAL = 1.5  # m
SAME_CAR_SPEED = 4.0  # m/s
HOLD_PATH_MARGIN = 2.0  # m from the model's path to the camera's car
DIRECT, BLEND, HOLD = range(3)

# Ignore Stationary Radar Ghosts
GHOST_TRACK_MAX_SPEED = 1.0  # m/s: the radar point is standing still
GHOST_VISION_MIN_SPEED = 5.0  # m/s (11 mph): the camera's car is clearly moving
GHOST_VISION_BRAKING = -1.5  # m/s^2: the camera's car braking harder than this keeps the radar point


@dataclass(frozen=True)
class LeadSensitivityConfig:
  camera: float = DEFAULT_CONFIDENCE
  radar: float = DEFAULT_CONFIDENCE
  smooth: bool = False
  ignore_ghosts: bool = False

  @property
  def match(self) -> float:
    """Radar pairing starts at the lower of the two cutoffs."""
    return min(self.camera, self.radar)


def confidence_from_percent(value) -> float:
  """Setting in percent -> probability, clamped to 25-50%. Anything unreadable is stock radard's 0.5."""
  try:
    pct = float(value)
  except (TypeError, ValueError):
    return DEFAULT_CONFIDENCE
  if not math.isfinite(pct):
    return DEFAULT_CONFIDENCE
  return min(max(pct / 100., CONFIDENCE_LIMITS[0]), CONFIDENCE_LIMITS[1])


class LeadSensitivitySettings:
  """Reads the settings once per second. config is None while Custom Lead Sensitivity is off, and always when there
  are no params (tests and replay)."""

  def __init__(self, params=None):
    self.params = params
    self.config: LeadSensitivityConfig | None = None
    self.frame = -1
    if self.params is not None:
      self.read()

  def _flag(self, key: str) -> bool:
    try:
      return bool(self.params.get_bool(key))
    except Exception:
      return False

  def _confidence(self, key: str) -> float:
    try:
      return confidence_from_percent(self.params.get(key))
    except Exception:
      return DEFAULT_CONFIDENCE

  def read(self):
    if not self._flag(PARAM_ENABLED):
      self.config = None
      return
    self.config = LeadSensitivityConfig(camera=self._confidence(PARAM_CAMERA), radar=self._confidence(PARAM_RADAR),
                                        smooth=self._flag(PARAM_SMOOTH), ignore_ghosts=self._flag(PARAM_GHOSTS))

  def update(self):
    self.frame += 1
    if self.params is not None and self.frame % round(PARAMS_PERIOD / DT_MDL) == 0:
      self.read()


def stationary_ghost(track_v_lead: float, vision_v: float, vision_a: float) -> bool:
  """A standing-still radar point paired with a camera car that is clearly moving and not braking hard: they can't be
  the same object."""
  return abs(track_v_lead) < GHOST_TRACK_MAX_SPEED and vision_v > GHOST_VISION_MIN_SPEED and vision_a > GHOST_VISION_BRAKING


def same_car_distance_margin(distance: float) -> float:
  """4-10 m: 10% of the gap."""
  return max(4.0, min(10.0, 0.1 * distance))


def same_car(predicted: dict | None, measured: dict | None) -> bool:
  if predicted is None or measured is None:
    return False
  return (abs(measured['dRel'] - predicted['dRel']) < same_car_distance_margin(predicted['dRel']) and
          abs(measured['yRel'] - predicted['yRel']) < SAME_CAR_LATERAL and
          abs(measured['vRel'] - predicted['vRel']) < SAME_CAR_SPEED)


def track_fits_camera(track, lead_msg, v_ego: float, vision_distance: float) -> bool:
  """The radar point already being followed is still at, beside and moving with the camera's car. The distance check
  matters at long range: radard's own pairing allows 25% of the gap, so without it a car that cuts in 15-20 m closer
  could lose to the point already followed."""
  return (abs(track.dRel - vision_distance) < same_car_distance_margin(vision_distance) and
          abs(track.yRel + lead_msg.y[0]) < SAME_CAR_LATERAL and abs(track.vRel + v_ego - lead_msg.v[0]) < SAME_CAR_SPEED)


def camera_on_path(lead_msg, model_path) -> bool:
  if model_path is None:
    return True
  x, y = model_path.x, model_path.y
  distance = lead_msg.x[0]
  for i in range(1, min(len(x), len(y))):
    if x[i - 1] <= distance <= x[i] and x[i] > x[i - 1]:
      fraction = (distance - x[i - 1]) / (x[i] - x[i - 1])
      lateral = y[i - 1] + fraction * (y[i] - y[i - 1])
      return abs(lead_msg.y[0] - lateral) < HOLD_PATH_MARGIN
  return False


class LeadHandoff:
  """Smooth Lead Handoffs for one lead slot: the last measurement (to recognize the same car) and the last published
  estimate (to predict from)."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.measurement: dict | None = None
    self.estimate: dict | None = None
    self.ego_speed = 0.0
    self.lateral_speed = 0.0
    self.held_frames = 0
    self.blend_frames = 0
    self.phase = DIRECT

  def predict(self, lead: dict | None, ego_speed: float) -> dict | None:
    if lead is None:
      return None
    speed_change = lead['aLeadK'] * DT_MDL
    next_speed = max(0.0, lead['vLead'] + speed_change)
    relative_speed = lead['vRel'] + next_speed - lead['vLead'] - (ego_speed - self.ego_speed)
    return dict(lead, dRel=max(0.0, lead['dRel'] + 0.5 * DT_MDL * (lead['vRel'] + relative_speed)),
                vRel=relative_speed, vLead=next_speed, vLeadK=max(0.0, lead['vLeadK'] + speed_change),
                yRel=lead['yRel'] + self.lateral_speed * DT_MDL)

  def update(self, lead: dict, camera: dict | None, camera_supports_hold: bool, ego_speed: float, probability: float,
             reset: bool = False) -> dict:
    prediction = self.predict(self.estimate, ego_speed)
    reference = self.predict(self.measurement, ego_speed)
    self.phase = DIRECT
    if reset:
      self.reset()
      return lead

    if not lead['present']:
      if (self.measurement is None or not self.measurement['radar'] or not camera_supports_hold or
          self.held_frames >= HOLD_FRAMES or not same_car(reference, camera)):
        self.reset()
        return lead

      assert prediction is not None  # a measurement always comes with an estimate
      self.held_frames += 1
      self.phase = HOLD
      self.measurement = reference
      self.estimate = dict(prediction, modelProb=probability)
      self.ego_speed = ego_speed
      return self.estimate

    if not all(math.isfinite(lead[key]) for key in ('dRel', 'yRel', 'vRel', 'vLead', 'vLeadK', 'aLeadK', 'aLeadTau')):
      self.reset()
      return lead

    if same_car(reference, lead):
      assert self.measurement is not None
      source_changed = self.measurement['radar'] != lead['radar'] or self.measurement['radarTrackId'] != lead['radarTrackId']
      if source_changed or self.held_frames:
        self.lateral_speed = 0.0
        if not self.blend_frames or self.held_frames:
          self.blend_frames = BLEND_FRAMES
      else:
        lateral_speed = (lead['yRel'] - self.measurement['yRel']) / DT_MDL
        self.lateral_speed = 0.75 * self.lateral_speed + 0.25 * max(-2.5, min(2.5, lateral_speed))
    else:
      self.reset()

    out = lead
    if self.blend_frames:
      assert prediction is not None  # blending only continues the same car
      out = dict(lead)
      # Compare against the prediction, so an expected change in gap isn't mistaken for good or bad news.
      for field in ('dRel', 'vRel', 'vLead', 'vLeadK', 'aLeadK'):
        if lead[field] > prediction[field]:
          out[field] = prediction[field] + (lead[field] - prediction[field]) / self.blend_frames
      self.phase = BLEND
      self.blend_frames -= 1

    self.held_frames = 0
    self.measurement = lead
    self.estimate = out
    self.ego_speed = ego_speed
    return out
