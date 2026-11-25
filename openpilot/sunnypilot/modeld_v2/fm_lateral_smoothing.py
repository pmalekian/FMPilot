"""FM Pilot: Model Lateral Smoothing for small and big models (FM_SmallModelLatSmoothing, FM_BigModelLatSmoothing).

Small models (FM Pilot's own): replaces the small model's LAT_SMOOTH_SECONDS (default: bundle's `lat` override, or
stock modeld's 0.0). That one value both low-passes the model's curvature (post-FOF models) and is added to the lateral
delay handed to the model, so a higher value both calms steering jitter and steers slightly earlier.

Big models (ported from zoompilot's BigModelSmoothing, Zeph Leggett, zoompilot/zoompilot 6cb59d3, MIT): a big model
(comma's large model over Jetlink, or on a chestnut) runs comma's stock modeld constants where LAT_SMOOTH_SECONDS is
0.0: its curvature is not low-passed at all, which on straights can leave a ~1 Hz side-to-side wiggle. Selecting 0.10 s
gives a big model that smooths none itself the 0.1 s low-pass the small model bundles commonly carry. Unlike the
small-model setting, it is NOT added to the lateral delay the model is told about (a big model steers from its action
head, so shifting delay would not lead the smoothing's lag and feeds the model<->car loop); it only calms and answers
about 0.1 s later. A big model whose own LAT_SMOOTH_SECONDS is already above 0 is left alone, so nothing is smoothed
twice.

Below the lateral control speed every action function holds the previous curvature, which smoothing leaves as it is.

Drop-down modes for both keys:
  0 = Model Default (old toggle off)
  1 = 0.00 s
  2 = 0.10 s
"""
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value

PARAM_SMALL_MODEL_SMOOTHING = "FM_SmallModelLatSmoothing"
PARAM_BIG_MODEL_SMOOTHING = "FM_BigModelLatSmoothing"

MODE_DEFAULT = 0
MODE_0_00 = 1
MODE_0_10 = 2
VALID_MODES = (MODE_DEFAULT, MODE_0_00, MODE_0_10)

BIG_MODEL_LAT_SMOOTH_SECONDS = 0.1


def read_smoothing_mode(params, key: str) -> int:
  """Read a Model Lateral Smoothing drop-down mode, falling back to Model Default on unset or invalid values."""
  raw = params.get(key, return_default=True)
  if raw is None:
    return MODE_DEFAULT
  try:
    mode = int(raw)
  except (TypeError, ValueError):
    return MODE_DEFAULT
  return mode if mode in VALID_MODES else MODE_DEFAULT


def small_lat_smooth_seconds(mode: int, bundle_seconds: float) -> float:
  """LAT_SMOOTH_SECONDS for a small model: Model Default keeps the bundle/stock value; 0.00 s or 0.10 s overrides it."""
  if mode == MODE_0_00:
    return 0.0
  if mode == MODE_0_10:
    return 0.1
  return float(bundle_seconds)


def big_bundle_lat_smooth_seconds(mode: int, bundle_seconds: float) -> float:
  """LAT_SMOOTH_SECONDS on a modeld_v2 chestnut ModelState: 0.00 s clears any bundle override; Model Default and 0.10 s
  keep the bundle value so lat_delay is never shifted (0.10 s smoothing is applied post-action by smooth_curvature)."""
  if mode == MODE_0_00:
    return 0.0
  return float(bundle_seconds)


def big_lat_smooth_seconds(mode: int, model) -> float:
  """Seconds of curvature smoothing to add post-action for the model that produced this frame's output: 0.1 s for a
  big model that smooths none itself when FM_BigModelLatSmoothing is 0.10 s; 0 otherwise. Read after model.run(): with
  Jetlink, a swap or a hand-back happens inside run(), and the joining model's `chestnut` and `LAT_SMOOTH_SECONDS`
  then follow the model whose output it is."""
  if mode != MODE_0_10 or not getattr(model, 'chestnut', False):
    return 0.0
  if getattr(model, 'LAT_SMOOTH_SECONDS', 0.0) > 0.0:
    return 0.0
  return BIG_MODEL_LAT_SMOOTH_SECONDS


def smooth_curvature(action, prev_action, seconds: float) -> None:
  """Low-pass the action's curvature by `seconds`, as modeld_v2 smooths a bundle's."""
  if seconds > 0.0:
    action.desiredCurvature = float(smooth_value(action.desiredCurvature, prev_action.desiredCurvature, seconds))
