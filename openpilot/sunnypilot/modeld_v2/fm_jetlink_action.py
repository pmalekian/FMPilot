"""
FM Pilot: the action function a big model joining over Jetlink runs through.

From zoompilot's modeld_v2/model_settings.py `with_blendful` (Zeph Leggett, zoompilot develop 92e1ecb, Nov 2026, MIT).
modeld_v2 calls get_action_from_model with Blendful's callback as a sixth argument; stock modeld's takes five. Stock
leaves the curvature unsmoothed (LAT_SMOOTH_SECONDS 0), so the correction added to its answer is the one modeld_v2 adds
before smoothing. Blendful returns exactly 0.0 unless FM_LaneKeepMode selects it.
"""


def with_blendful(get_action_from_model, min_lat_control_speed: float):
  def action(model_output, prev_action, lat_action_t, long_action_t, v_ego, blendful=None):
    action = get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego)
    if blendful is not None:
      correction = blendful(action.desiredCurvature)
      if v_ego > min_lat_control_speed:
        action.desiredCurvature += correction
    return action
  return action
