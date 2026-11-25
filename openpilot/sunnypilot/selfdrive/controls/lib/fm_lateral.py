"""
FM Pilot: lateral additions applied to the requested curvature in controlsd, after AP's lane centering and before AP's
rear-axle off-tracking and clip_curvature, so they work the same with torque and angle steering.

  - Lane Guard (from gerrylum's rx-wb, with its Rivian tune; selfdrive/controls/lib/lane_guard.py): a small correction
    only when the planned path brings the body too close to a lane line or road edge. One choice of the Lane Keeping
    selector (FM_LaneKeepMode, fm_lane_keep_mode.py), so it never runs together with lane centering or Blendful.
  - Turn Hold and Smooth Turn Start (fm_turn_assist.py): a steadier request during low-speed blinker turns. Applied
    first, so Lane Guard and AP's off-tracking still apply to the result.

Ported from fm-dev-nav-settle ca08aac9 (FM's own copy of gerrylum's off-tracking is dropped: AP's rivian_offtrack runs
after this).
"""
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.lane_guard import LANE_CHANGE_COOLDOWN, STOCK, TUNES, LaneGuard
from openpilot.sunnypilot.selfdrive.controls.lib.fm_lane_keep_mode import MODE_GUARD, get_lane_keep_mode
from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import FmTurnAssist

PARAMS_PERIOD = 1.0  # s


class FmLateral:
  def __init__(self, CP, params):
    self.CP = CP
    self.params = params
    self.turn_assist = FmTurnAssist(params)
    self.lane_guard = LaneGuard(TUNES.get(CP.brand, STOCK))
    self.lane_guard_enabled = False
    self.lane_guard_resting = False  # Lane Guard is off and has faded out: its update is skipped
    self.left_hand_traffic = False
    self.frame = 0
    self.read_params()

  def read_params(self) -> None:
    self.lane_guard_enabled = get_lane_keep_mode(self.params) == MODE_GUARD
    self.left_hand_traffic = self.params.get_bool("IsRhdDetected")
    if hasattr(self, 'turn_assist'):
      self.turn_assist.read_params()

  def update(self, desired_curvature: float, model_v2, CS, lat_active: bool, lane_centering_enabled: bool,
             actual_curvature: float = 0.0, sm=None) -> float:
    self.frame += 1
    if self.frame % int(PARAMS_PERIOD / DT_CTRL) == 0:
      self.read_params()

    # Turn Assist reads the active turn desire (modelDataV2SP.laneTurnDirection, from modeld) from controlsd's own
    # SubMaster, already updated this step: a SubMaster polled here would sleep inside the control step whenever no new
    # 20 Hz message is waiting (fm-dev-nav-settle measured 68 us a step on a PC).
    if self.turn_assist.enabled:
      turn_direction = None
      if sm is not None and sm.seen['modelDataV2SP']:
        turn_direction = sm['modelDataV2SP'].laneTurnDirection
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
    return desired_curvature + guard
