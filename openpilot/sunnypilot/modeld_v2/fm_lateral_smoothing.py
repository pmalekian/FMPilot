"""FM Pilot: model smoothing settings ported from fm-dev-nav-settle (ca08aac9).

Big-model post-filter ported from zoompilot's BigModelSmoothing
(Zeph Leggett, zoompilot/zoompilot 6cb59d3, MIT).
"""
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value

PARAM_SMALL_MODEL = "FM_SmallModelSmoothingMode"
PARAM_BIG_MODEL = "FM_BigModelSmoothingMode"

MODEL_DEFAULT = 0
ZERO_SECONDS = 1
TENTH_SECOND = 2
OVERRIDE_SECONDS = {ZERO_SECONDS: 0.0, TENTH_SECOND: 0.1}


def smoothing_override(params, key: str, default: float | None = None) -> float | None:
  """An explicit override in seconds, or the model's default."""
  return OVERRIDE_SECONDS.get(params.get(key, return_default=True), default)


def smooth_big_model_action(action, prev_action, model, seconds: float | None) -> None:
  """Filter the big model's curvature after inference, including a Jetlink handover."""
  if seconds and model.chestnut:
    action.desiredCurvature = float(smooth_value(action.desiredCurvature, prev_action.desiredCurvature, seconds))
