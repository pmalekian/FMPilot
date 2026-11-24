"""
Custom Lead Sensitivity: settings for when radard starts following the camera's lead car.

Stock radard follows the driving model's lead car, and tries to pair it with a radar point, only once the model is more
than 50% sure of it. With Custom Lead Sensitivity on, two settings replace that fixed cutoff:

- Camera-only Lead Confidence: how sure the model must be to follow a car that no radar point confirms.
- Radar-matched Lead Confidence: how sure the model must be before radard tries to pair the car with a radar point.

Radar pairing starts at the lower of the two, so a car the radar confirms can be followed sooner than one it doesn't.
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

DEFAULT_CONFIDENCE = 0.5  # stock radard's cutoff
CONFIDENCE_LIMITS = (0.25, 0.5)
PARAMS_PERIOD = 1.0  # s


@dataclass(frozen=True)
class LeadSensitivityConfig:
  camera: float = DEFAULT_CONFIDENCE
  radar: float = DEFAULT_CONFIDENCE

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
    self.config = LeadSensitivityConfig(camera=self._confidence(PARAM_CAMERA), radar=self._confidence(PARAM_RADAR))

  def update(self):
    self.frame += 1
    if self.params is not None and self.frame % round(PARAMS_PERIOD / DT_MDL) == 0:
      self.read()
