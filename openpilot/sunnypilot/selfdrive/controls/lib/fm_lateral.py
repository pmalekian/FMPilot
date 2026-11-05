"""
FM Pilot: lateral additions applied to the requested curvature in controlsd, after AP's lane centering and before
clip_curvature, so they work the same with torque and angle steering.

  - Lane Guard (from gerrylum's rx-wb, with its Rivian tune): a small correction only when the planned path brings the
    body too close to a lane line or road edge. One choice of FM_LaneKeepMode (fm_lane_keep_mode.py), so it never runs
    together with lane centering or Blendful.
  - Rear-axle off-tracking compensation (from rx-wb): widens turns by gain x the rear axle's cut-in. Rivian only.
  - Turn Boost, Turn Lead and Turn Hold (fm_turn_assist.py): stronger, earlier, steadier requests during low-speed
    blinker turns. Applied first, so off-tracking (if on) still widens the result.
"""
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.lane_guard import STOCK, TUNES, LaneGuard
from openpilot.sunnypilot.selfdrive.controls.lib.fm_lane_keep_mode import MODE_GUARD, get_lane_keep_mode
from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import FmTurnAssist
from openpilot.sunnypilot.selfdrive.controls.lib.rivian_offtrack import offtrack_curvature

PARAM_OFFTRACK = "RivianOfftrackCompensationEnabled"
PARAM_OFFTRACK_GAIN = "RivianOfftrackCompensationGain"
PARAMS_PERIOD = 1.0  # s


class FmLateral:
  def __init__(self, CP, params, turn_sm=None):
    self.CP = CP
    self.params = params
    self.turn_assist = FmTurnAssist(params)
    if turn_sm is None:
      from openpilot.cereal import messaging
      turn_sm = messaging.SubMaster(['modelDataV2SP'])  # the active turn desire, published by modeld
    self.turn_sm = turn_sm
    self.lane_guard = LaneGuard(TUNES.get(CP.brand, STOCK))
    self.lane_guard_enabled = False
    self.offtrack_gain = 0.0
    self.left_hand_traffic = False
    self.frame = 0
    self.read_params()

  def read_params(self) -> None:
    self.lane_guard_enabled = get_lane_keep_mode(self.params) == MODE_GUARD
    self.left_hand_traffic = self.params.get_bool("IsRhdDetected")
    gain = 0.0
    if self.CP.brand == 'rivian' and self.params.get_bool(PARAM_OFFTRACK):
      gain = float(self.params.get(PARAM_OFFTRACK_GAIN, return_default=True) or 0.0)
    self.offtrack_gain = gain
    if hasattr(self, 'turn_assist'):
      self.turn_assist.read_params()

  def update(self, desired_curvature: float, model_v2, CS, lat_active: bool, lane_centering_enabled: bool,
             actual_curvature: float = 0.0) -> float:
    self.frame += 1
    if self.frame % int(PARAMS_PERIOD / DT_CTRL) == 0:
      self.read_params()

    if self.turn_assist.enabled:
      self.turn_sm.update(0)
      turn_direction = self.turn_sm['modelDataV2SP'].laneTurnDirection if self.turn_sm.seen['modelDataV2SP'] else None
      lane_change_state = getattr(getattr(model_v2, 'meta', None), 'laneChangeState', 0)
      desired_curvature = self.turn_assist.update(desired_curvature, turn_direction, CS.vEgo, actual_curvature,
                                                  CS.steeringPressed, lat_active, model_v2=model_v2,
                                                  a_ego=float(getattr(CS, 'aEgo', 0.0)),
                                                  lane_changing=lane_change_state != 0,
                                                  steering_torque=float(getattr(CS, 'steeringTorque', 0.0)))

    # Lane Guard and lane centering are exclusive: lane centering wins if both are on
    guard_active = self.lane_guard_enabled and not lane_centering_enabled and lat_active and not CS.steeringPressed
    guard = self.lane_guard.update(CS.vEgo, model_v2, guard_active, bool(CS.leftBlinker or CS.rightBlinker), self.left_hand_traffic)
    if not lat_active:
      return desired_curvature

    desired_curvature += guard
    return offtrack_curvature(desired_curvature, self.CP.wheelbase, self.offtrack_gain)
