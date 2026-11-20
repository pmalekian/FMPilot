"""FM Pilot: Lateral Smoothing (Big Models), FM_BigModelSmoothing.

Ported from zoompilot's BigModelSmoothing (Zeph Leggett, zoompilot/zoompilot 6cb59d3, MIT). A big model (comma's large
model over Jetlink, or on a chestnut) runs comma's stock modeld constants, and LAT_SMOOTH_SECONDS there is 0.0: its
curvature is not low-passed at all, which on straights can leave a ~1 Hz side-to-side wiggle (zoompilot's 11-11 drives).
With this on, a big model that smooths nothing itself gets the 0.1 s low-pass the small model bundles commonly carry.

Unlike a bundle's own value (and FM_LateralSmoothingEnabled, which is for the small models), it is NOT added to the
lateral delay the model is told about: a big model steers from its action head, so that shift would not lead the
smoothing's lag, and telling it the car is slower than lagd measured feeds the model<->car loop the smoothing is meant
to damp. A big model whose own LAT_SMOOTH_SECONDS is already above 0 is left alone, so nothing is smoothed twice.

Below the lateral control speed every action function holds the previous curvature, which smoothing leaves as it is.
"""
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value

PARAM_BIG_MODEL_SMOOTHING = "FM_BigModelSmoothing"
BIG_MODEL_LAT_SMOOTH_SECONDS = 0.1


def big_lat_smooth_seconds(enabled: bool, model) -> float:
  """Seconds of curvature smoothing to add for the model that produced this frame's output: the small models' common
  amount for a big model that smooths none itself, with the setting on; 0 otherwise. Read after model.run(): with
  Jetlink, a swap or a hand-back happens inside run(), and the joining model's `chestnut` and `LAT_SMOOTH_SECONDS`
  then follow the model whose output it is."""
  if not (enabled and getattr(model, 'chestnut', False)):
    return 0.0
  if getattr(model, 'LAT_SMOOTH_SECONDS', 0.0) > 0.0:
    return 0.0
  return BIG_MODEL_LAT_SMOOTH_SECONDS


def smooth_curvature(action, prev_action, seconds: float) -> None:
  """Low-pass the action's curvature by `seconds`, as modeld_v2 smooths a bundle's."""
  if seconds > 0.0:
    action.desiredCurvature = float(smooth_value(action.desiredCurvature, prev_action.desiredCurvature, seconds))
