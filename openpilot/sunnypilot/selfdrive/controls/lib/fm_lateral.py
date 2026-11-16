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
from openpilot.selfdrive.controls.lib.lane_guard import LANE_CHANGE_COOLDOWN, STOCK, TUNES, LaneGuard
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
    # The active turn desire (modelDataV2SP.laneTurnDirection, published by modeld) comes from the SubMaster controlsd
    # already runs, passed to update(). This used to be a SubMaster of its own, updated inside the control step:
    # msgq's poll sleeps (nanosleep) whenever no new message is waiting, and modelDataV2SP is 20 Hz, so controlsd went
    # to sleep in the middle of 4 of every 5 steps (68 us a step on a PC, against 0.3 us for one more service on
    # controlsd's own SubMaster). turn_sm is only a fallback for callers that pass none (tests).
    self.turn_sm = turn_sm
    self.lane_guard = LaneGuard(TUNES.get(CP.brand, STOCK))
    self.lane_guard_enabled = False
    self.lane_guard_resting = False  # Lane Guard is off and has faded out: its update is skipped
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
             actual_curvature: float = 0.0, sm=None) -> float:
    self.frame += 1
    if self.frame % int(PARAMS_PERIOD / DT_CTRL) == 0:
      self.read_params()

    if self.turn_assist.enabled:
      turn_sm = sm if sm is not None else self.turn_sm  # already updated by its owner this step; never polled here
      turn_direction = None
      if turn_sm is not None and turn_sm.seen['modelDataV2SP']:
        turn_direction = turn_sm['modelDataV2SP'].laneTurnDirection
      lane_change_state = getattr(getattr(model_v2, 'meta', None), 'laneChangeState', 0)
      desired_curvature = self.turn_assist.update(desired_curvature, turn_direction, CS.vEgo, actual_curvature,
                                                  CS.steeringPressed, lat_active, model_v2=model_v2,
                                                  a_ego=float(getattr(CS, 'aEgo', 0.0)),
                                                  lane_changing=lane_change_state != 0,
                                                  steering_torque=float(getattr(CS, 'steeringTorque', 0.0)))

    # Lane Guard and lane centering are exclusive: lane centering wins if both are on.
    # While Lane Guard isn't the selected mode and has nothing left to fade out, it isn't run at all: its update reads
    # the model's lane lines and edges on every control step only to return 0 (15 us a step on a PC).
    guard = 0.0
    if self.lane_guard_enabled or self.lane_guard.curvature != 0.0 or self.lane_guard.target != 0.0:
      if self.lane_guard_resting:
        # switched on while driving: it hasn't been watching the lines or the blinker, so start as after a lane
        # change (lines unguarded for a second, then trusted only above the higher probability)
        self.lane_guard.lane_change_timer = LANE_CHANGE_COOLDOWN
        self.lane_guard.line_valid = [False, False]
        self.lane_guard_resting = False
      guard_active = self.lane_guard_enabled and not lane_centering_enabled and lat_active and not CS.steeringPressed
      guard = self.lane_guard.update(CS.vEgo, model_v2, guard_active, bool(CS.leftBlinker or CS.rightBlinker), self.left_hand_traffic)
    else:
      self.lane_guard_resting = True
    if not lat_active:
      return desired_curvature

    desired_curvature += guard
    return offtrack_curvature(desired_curvature, self.CP.wheelbase, self.offtrack_gain)
