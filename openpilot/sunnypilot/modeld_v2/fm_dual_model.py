"""
FM Pilot: dual model support for modeld_v2.

One model steers (the lateral slot) and another handles speed (the longitudinal slot). Both run on the same camera
frames with the same desire; their outputs are merged into the one output dict that fill_model_msg already knows how
to publish, so nothing downstream of modeld needs to know two models ran.

Merge rules (see merge_outputs):
  - Path geometry (lateral offset, height, heading as a function of distance) comes from the lateral model, traversed
    at the longitudinal model's speed plan. position.x / velocity / acceleration therefore come from the longitudinal
    model, which keeps DEC's slow-down check and the e2e alerts on the model that controls speed, while lane centering,
    Lane Guard and road-edge detection see the lateral model's path.
  - Lane lines, road edges, desire state (lane changes) and camera pose: lateral model.
  - Leads, meta (disengage, gas/brake press, hard-brake/FCW probabilities): longitudinal model.
  - Curve speed: with the "lateral" curve source the turn rate is the lateral path's curvature at the longitudinal
    model's speed (so Smart Cruise sees the curve the car will actually take); with "longitudinal" it is the
    longitudinal model's own turn rate.
"""
import re

import numpy as np

from openpilot.cereal import custom, log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.modeld_v2.constants import Plan

PARAM_ENABLED = "FM_DualModelEnabled"
PARAM_LAT_REF = "FM_LateralModelRef"
PARAM_LONG_REF = "FM_LongModelRef"
PARAM_CURVE_SOURCE = "FM_CurveSpeedSource"
PARAM_SHOW_LABEL = "FM_ShowModelLabel"

CURVE_SOURCE_LATERAL = 0
CURVE_SOURCE_LONGITUDINAL = 1

# A modeld step has 50 ms. Both models together must leave room for the rest of the loop.
MAX_DUAL_EXECUTION_TIME = 0.045  # s
SLOW_FALLBACK_TIME = 5.0  # s of sustained over-budget running before falling back to the lateral model alone
FALLBACK_NOTICE_TIME = 10.0  # s the on-screen fallback notice is shown

MIN_PATH_LENGTH = 1.0  # m, below this the lateral path is used as is (standstill)

# keys taken from the lateral model; everything else comes from the longitudinal model
LATERAL_KEYS = ('lane_lines', 'lane_lines_stds', 'lane_lines_prob', 'road_edges', 'road_edges_stds',
                'desire_state', 'desire_pred', 'desired_curvature',
                'pose', 'pose_stds', 'wide_from_device_euler', 'wide_from_device_euler_stds',
                'road_transform', 'road_transform_stds')
# keys both models must produce with the same shape for the merge to be valid
REQUIRED_KEYS = ('plan', 'plan_stds', 'lane_lines', 'lane_lines_stds', 'lane_lines_prob', 'road_edges',
                 'road_edges_stds', 'lead', 'lead_stds', 'lead_prob', 'desire_state', 'meta')


def _bundle_ref(bundle) -> str:
  return bundle.ref if bundle is not None else ""


def resolve_slot_bundles(params, selected_bundle, cached_bundles):
  """Returns (lateral_bundle, longitudinal_bundle). An empty or unknown slot falls back to the selected bundle."""
  by_ref = {b.ref: b for b in cached_bundles if b.ref}
  if selected_bundle is not None and selected_bundle.ref:
    by_ref.setdefault(selected_bundle.ref, selected_bundle)

  def slot(key):
    ref = params.get(key) or ""
    return by_ref.get(ref, selected_bundle) if ref else selected_bundle

  return slot(PARAM_LAT_REF), slot(PARAM_LONG_REF)


def check_compatible(lat_output: dict, long_output: dict) -> str | None:
  """Returns None when the two models' outputs can be merged, otherwise the reason they can't."""
  for key in REQUIRED_KEYS:
    if key not in lat_output or key not in long_output:
      return f"missing output {key}"
    if lat_output[key].shape != long_output[key].shape:
      return f"output {key} shape {lat_output[key].shape} != {long_output[key].shape}"
  return None


def _monotonic(x: np.ndarray) -> np.ndarray:
  return np.maximum.accumulate(x)


def merge_plan(lat_plan: np.ndarray, long_plan: np.ndarray, curve_source: int) -> np.ndarray:
  """Lateral model's path geometry traversed at the longitudinal model's speed. Shapes: (1, IDX_N, PLAN_WIDTH)."""
  merged = long_plan.copy()
  lat_pos, long_pos = lat_plan[0, :, Plan.POSITION], long_plan[0, :, Plan.POSITION]
  lat_x = _monotonic(lat_pos[:, 0])
  long_x = long_pos[:, 0]

  if lat_x[-1] - lat_x[0] < MIN_PATH_LENGTH:
    # (nearly) standing still: nothing to resample, take the lateral path and heading as they are
    merged[0, :, Plan.POSITION][:, 1:] = lat_pos[:, 1:]
    merged[0, :, Plan.T_FROM_CURRENT_EULER] = lat_plan[0, :, Plan.T_FROM_CURRENT_EULER]
    if curve_source == CURVE_SOURCE_LATERAL:
      merged[0, :, Plan.ORIENTATION_RATE] = lat_plan[0, :, Plan.ORIENTATION_RATE]
    return merged

  # lateral offset and height as a function of distance travelled
  for axis in (1, 2):
    merged[0, :, Plan.POSITION.start + axis] = np.interp(long_x, lat_x, lat_pos[:, axis])
  # heading (roll, pitch, yaw) as a function of distance
  lat_euler = lat_plan[0, :, Plan.T_FROM_CURRENT_EULER]
  for axis in range(3):
    merged[0, :, Plan.T_FROM_CURRENT_EULER.start + axis] = np.interp(long_x, lat_x, lat_euler[:, axis])

  if curve_source == CURVE_SOURCE_LATERAL:
    # yaw rate = path curvature at that distance * longitudinal speed
    lat_v = np.maximum(lat_plan[0, :, Plan.VELOCITY][:, 0], 0.1)
    lat_kappa = lat_plan[0, :, Plan.ORIENTATION_RATE][:, 2] / lat_v
    long_v = long_plan[0, :, Plan.VELOCITY][:, 0]
    merged[0, :, Plan.ORIENTATION_RATE.start + 2] = np.interp(long_x, lat_x, lat_kappa) * long_v
  return merged


def merge_outputs(lat_output: dict, long_output: dict, curve_source: int) -> dict:
  merged = dict(long_output)
  for key in LATERAL_KEYS:
    if key in lat_output:
      merged[key] = lat_output[key]
  merged['plan'] = merge_plan(lat_output['plan'], long_output['plan'], curve_source)
  # path uncertainty follows the path's owner
  plan_stds = long_output['plan_stds'].copy()
  plan_stds[0, :, Plan.POSITION][:, 1:] = lat_output['plan_stds'][0, :, Plan.POSITION][:, 1:]
  merged['plan_stds'] = plan_stds
  return merged


def merge_actions(lat_action, long_action):
  return log.ModelDataV2.Action(desiredCurvature=lat_action.desiredCurvature,
                                desiredAcceleration=long_action.desiredAcceleration,
                                shouldStop=long_action.shouldStop)


def feed_back_curvature(long_model, lat_output: dict) -> None:
  """The longitudinal model gets the curvature the car actually followed (the lateral model's) as its history."""
  buf = long_model.numpy_inputs.get('prev_desired_curv')
  if buf is None or long_model.mlsim or 'desired_curvature' not in lat_output:
    return
  try:
    buf[0, -1, :] = np.asarray(lat_output['desired_curvature'][0, :]).reshape(buf[0, -1, :].shape)
  except ValueError:
    pass


def short_name(display_name: str, max_len: int = 16) -> str:
  """'Terrible Super Fantastic Do Over Model (August 15, 2026)' -> 'TSFDO', 'OP Model 16 Deep (June 03, 2026)' -> 'OP 16 Deep'."""
  name = re.sub(r"\s*\(.*?\)", "", display_name or "").strip()
  name = re.sub(r"\bmodel\b", "", name, flags=re.IGNORECASE)
  name = re.sub(r"\s+", " ", name).strip()
  if len(name) <= max_len:
    return name
  return "".join(w if w.isdigit() else w[0].upper() for w in name.split())


class DualModelStatus:
  """Tracks whether dual mode is running, and when to show the fallback notice."""
  def __init__(self):
    self.requested = False
    self.active = False
    self.fallback_reason = ""
    self.notice_timer = 0.0
    self.exec_time = FirstOrderFilter(0.0, SLOW_FALLBACK_TIME, DT_MDL)
    self.lat_name = ""
    self.long_name = ""

  def fall_back(self, reason: str) -> None:
    self.active = False
    self.fallback_reason = reason
    self.notice_timer = FALLBACK_NOTICE_TIME

  def update(self, execution_time: float) -> bool:
    """Call once per step while active. Returns True when the pair is too slow and should fall back."""
    self.notice_timer = max(self.notice_timer - DT_MDL, 0.0)
    if not self.active:
      return False
    return self.exec_time.update(execution_time) > MAX_DUAL_EXECUTION_TIME

  def fill(self, mdv2sp: custom.ModelDataV2SP) -> None:
    mdv2sp.fmDualModelActive = self.active
    mdv2sp.fmDualModelFallback = self.notice_timer > 0.0
    mdv2sp.fmLateralModelName = self.lat_name
    mdv2sp.fmLongModelName = self.long_name
    mdv2sp.fmDualModelReason = "" if self.active else self.fallback_reason


SAME_AS_SELECTED_LABEL = "Same as selected model"


def _bundle_downloaded(bundle, model_root: str) -> bool:
  import os
  from openpilot.common.file_chunker import get_manifest_path
  for model in getattr(bundle, 'models', []) or []:
    file_name = getattr(getattr(model, 'artifact', None), 'fileName', '')
    if not file_name:
      continue
    path = os.path.join(model_root, file_name)
    if not (os.path.exists(path) or os.path.exists(get_manifest_path(path))):
      return False
  return True


def build_slot_options(favorites: str | None, bundles, model_root: str, current_refs: tuple[str, ...] = ()) -> list[dict]:
  """Sunnylink dropdown options for a model slot: "Same as selected model", then the downloaded favorites. A slot's
  current value is always kept in the list so the dropdown can show it."""
  favorite_refs = {ref for ref in (favorites or "").split(';') if ref}
  options = [{"value": "", "label": SAME_AS_SELECTED_LABEL}]
  for bundle in sorted(bundles, key=lambda b: b.index, reverse=True):
    if not bundle.ref or not (bundle.ref in favorite_refs or bundle.ref in current_refs):
      continue
    if bundle.ref in favorite_refs and not _bundle_downloaded(bundle, model_root):
      continue
    options.append({"value": bundle.ref, "label": bundle.displayName or bundle.ref})
  return options


def get_slot_options() -> list[dict]:
  """Options from the device's params and model cache; just the default option when they can't be read."""
  try:
    from openpilot.common.hardware.hw import Paths
    from openpilot.common.params import Params
    from openpilot.sunnypilot.models.fetcher import get_cached_bundles
    params = Params()
    current = tuple(params.get(k) or "" for k in (PARAM_LAT_REF, PARAM_LONG_REF))
    return build_slot_options(params.get("ModelManager_Favs"), get_cached_bundles(params, "qcom"), Paths.model_root(), current)
  except Exception:
    return [{"value": "", "label": SAME_AS_SELECTED_LABEL}]


def label_text(mdv2sp, execution_time: float) -> str:
  """The on-screen model label: ASCII only (the UI font has no middle dot), short enough for the comma 4 screen.
  Ends with the model step's live run time, the number that decides whether two models fit."""
  lat, long = short_name(mdv2sp.fmLateralModelName), short_name(mdv2sp.fmLongModelName)
  if not lat:
    return ""
  ms = f"{execution_time * 1000:.0f} ms"
  if mdv2sp.fmDualModelActive:
    return f"L: {lat} | G: {long} | {ms}"
  reason = mdv2sp.fmDualModelReason
  if reason:
    timing = re.search(r"\((\d+) ms", reason)
    reason = f"pair {timing.group(1)} ms" if reason.startswith("too slow") and timing else reason[:24]
    return f"{lat} | {ms} | dual off: {reason}"
  return f"{lat} | {ms}"
