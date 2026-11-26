"""
FM Pilot: the action function a big model joining over Jetlink runs through.

From zoompilot's modeld_v2/model_settings.py `with_blendful` (Zeph Leggett, zoompilot develop 92e1ecb, Nov 2026, MIT).
modeld_v2 passes its Blendful and Settle After Turn callback as a sixth argument. Stock modeld computes the
unsmoothed action and receives the caller's crawl-speed floor. modeld_v2 applies optional big-model smoothing
after this wrapper returns its corrected curvature.
"""

from openpilot.sunnypilot.modeld_v2.fm_pull_away import ACTION_SPEED_FLOOR


def with_blendful(get_action_from_model, min_lat_control_speed: float):
  def action(model_output, prev_action, lat_action_t, long_action_t, v_ego, lane_keep=None, *, action_speed_floor=ACTION_SPEED_FLOOR):
    action = get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego,
                                   action_speed_floor=action_speed_floor)
    if lane_keep is not None:
      correction = lane_keep(action.desiredCurvature)
      if correction and v_ego > min_lat_control_speed:
        action.desiredCurvature += correction
    return action
  return action
