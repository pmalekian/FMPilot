"""
FM Pilot: lateral additions applied to the requested curvature in controlsd, after AP's lane centering and before AP's
rear-axle off-tracking and clip_curvature, so they work the same with torque and angle steering.

  - Lane Guard (from gerrylum's rx-wb, with its Rivian tune; selfdrive/controls/lib/lane_guard.py): a small correction
    only when the planned path brings the body too close to a lane line or road edge. One choice of the Lane Keeping
    selector (FM_LaneKeepMode, fm_lane_keep_mode.py), so it never runs together with lane centering or Blendful.
  - Turn Lead, Turn Hold and Smooth Turn Start (fm_turn_assist.py): a turn started on time and a steadier request
    during low-speed blinker turns. Applied first, so Lane Guard and AP's off-tracking still apply to the result.

Ported from fm-dev-nav-settle ca08aac9 (FM's own copy of gerrylum's off-tracking is dropped: AP's rivian_offtrack runs
after this).
"""
import math

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.lane_guard import LANE_CHANGE_COOLDOWN, STOCK, TUNES, LaneGuard
from openpilot.sunnypilot.selfdrive.controls.lib.fm_lane_keep_mode import MODE_GUARD, get_lane_keep_mode
from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import SOURCE_NONE, FmTurnAssist

PARAMS_PERIOD = 1.0  # s
CARRY_FADE_TIME = 0.2  # s, quick fade to exact zero for already-applied Lane Guard correction during a carry
CARRY_FADE_FRAMES = max(int(round(CARRY_FADE_TIME / DT_CTRL)), 1)


def settle_active_from_sm(sm) -> bool:
  """True when modeld's fmModel reports Settle After Turn / Early Right Push active."""
  if sm is None or not sm.seen.get('fmModel', False):
    return False
  fm = sm['fmModel']
  phase = getattr(fm, 'turnSettlePhase', 0)
  if str(phase) not in ('idle', '0', 'None'):
    return True
  curv = float(getattr(fm, 'turnSettle', 0.0))
  return math.isfinite(curv) and curv != 0.0


def _turn_inputs(model_v2, sm):
  turn_direction = None
  if sm is not None and sm.seen.get('modelDataV2SP', False):
    turn_direction = sm['modelDataV2SP'].laneTurnDirection
  lane_change_state = getattr(getattr(model_v2, 'meta', None), 'laneChangeState', 0)
  # Queue Guard (modeld's fmModel): a hold hides the turn desire, and is never a carry
  queue_hold = bool(sm is not None and sm.seen.get('fmModel', False) and sm['fmModel'].queueGuard)
  return turn_direction, lane_change_state != 0, queue_hold


class FmLateral:
  def __init__(self, CP, params):
    self.CP = CP
    self.params = params
    self.turn_assist = FmTurnAssist(params)
    self.lane_guard = LaneGuard(TUNES.get(CP.brand, STOCK))
    self.lane_guard_enabled = False
    self.lane_guard_resting = False  # Lane Guard is off and has faded out: its update is skipped
    self.lane_guard_carrying = False
    self.lane_guard_carry_frames = 0
    self.lane_guard_carry_start = 0.0
    self.left_hand_traffic = False
    self.frame = 0
    self.read_params()

  def read_params(self) -> None:
    self.lane_guard_enabled = get_lane_keep_mode(self.params) == MODE_GUARD
    self.left_hand_traffic = self.params.get_bool("IsRhdDetected")
    if hasattr(self, 'turn_assist'):
      self.turn_assist.read_params()

  def carry_active(self, model_v2, CS, lat_active: bool, actual_curvature: float = 0.0, sm=None) -> bool:
    """Read-only check: True while Finish Turn After Signal Cancels is carrying a turn this control step.
    Used by controlsd before lane_centering.update so AP Lane Centering stays out during a carry."""
    if not self.turn_assist.enabled:
      return False
    turn_direction, lane_changing, queue_hold = _turn_inputs(model_v2, sm)
    return self.turn_assist.will_carry(turn_direction, CS.vEgo, actual_curvature, CS.steeringPressed,
                                       float(getattr(CS, 'steeringTorque', 0.0)), lat_active,
                                       lane_changing=lane_changing, queue_hold=queue_hold)

  def update(self, desired_curvature: float, model_v2, CS, lat_active: bool, lane_centering_enabled: bool,
             actual_curvature: float = 0.0, sm=None) -> float:
    self.frame += 1
    if self.frame % int(PARAMS_PERIOD / DT_CTRL) == 0:
      self.read_params()

    # Turn Assist reads the active turn desire (modelDataV2SP.laneTurnDirection, from modeld) from controlsd's own
    # SubMaster, already updated this step: a SubMaster polled here would sleep inside the control step whenever no new
    # 20 Hz message is waiting (fm-dev-nav-settle measured 68 us a step on a PC).
    if self.turn_assist.enabled:
      turn_direction, lane_changing, queue_hold = _turn_inputs(model_v2, sm)
      desired_curvature = self.turn_assist.update(desired_curvature, turn_direction, CS.vEgo, actual_curvature,
                                                  CS.steeringPressed, lat_active, model_v2=model_v2,
                                                  a_ego=float(getattr(CS, 'aEgo', 0.0)),
                                                  lane_changing=lane_changing,
                                                  steering_torque=float(getattr(CS, 'steeringTorque', 0.0)),
                                                  queue_hold=queue_hold,
                                                  settle_active=settle_active_from_sm(sm))
    elif (self.turn_assist.sign != 0 or self.turn_assist.carry_sign != 0
          or self.turn_assist.log_source != SOURCE_NONE or self.turn_assist.amount != 0.0
          or self.turn_assist.log_exit != 0.0):
      self.turn_assist.reset()

    # Lane Guard and lane centering are exclusive: lane centering wins if both are on.
    # While Lane Guard isn't the selected mode and has nothing left to fade out, it isn't run at all: its update reads
    # the model's lane lines and edges on every control step only to return 0 (15 us a step on a PC).
    carrying = self.turn_assist.enabled and self.turn_assist.carry_sign != 0
    if not carrying:
      self.lane_guard_carrying = False
      self.lane_guard_carry_frames = 0
      self.lane_guard_carry_start = 0.0
    guard = 0.0
    if self.lane_guard_enabled or self.lane_guard.curvature != 0.0 or self.lane_guard.target != 0.0:
      if self.lane_guard_resting:
        # switched on while driving: it hasn't been watching the lines or the blinker, so start as after a lane
        # change (lines unguarded for a second, then trusted only above the higher probability)
        self.lane_guard.lane_change_timer = LANE_CHANGE_COOLDOWN
        self.lane_guard.line_valid = [False, False]
        self.lane_guard_resting = False
      if carrying:
        # FM Pilot: fade any already-applied Lane Guard correction (including a live road-edge push) to zero over ~0.2 s
        # and keep the 1 s lane-line cooldown armed so it comes back the same way it does after a blinker turn.
        self.lane_guard.lane_change_timer = LANE_CHANGE_COOLDOWN
        self.lane_guard.line_valid = [False, False]
        if not self.lane_guard_carrying:
          self.lane_guard_carrying = True
          self.lane_guard_carry_frames = CARRY_FADE_FRAMES
          self.lane_guard_carry_start = self.lane_guard.curvature
        self.lane_guard_carry_frames = max(self.lane_guard_carry_frames - 1, 0)
        guard = (self.lane_guard_carry_start * (self.lane_guard_carry_frames / CARRY_FADE_FRAMES)
                 if self.lane_guard_carry_frames > 0 else 0.0)
        self.lane_guard.curvature = guard
        self.lane_guard.target = guard
      else:
        guard_active = (self.lane_guard_enabled and not lane_centering_enabled and lat_active
                        and not CS.steeringPressed)
        guard = self.lane_guard.update(CS.vEgo, model_v2, guard_active, bool(CS.leftBlinker or CS.rightBlinker),
                                       self.left_hand_traffic)
    else:
      self.lane_guard_resting = True
    if not lat_active:
      return desired_curvature
    return desired_curvature + guard
