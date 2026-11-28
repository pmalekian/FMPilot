"""
FM Pilot: the Accel Boost picker (FM_AccelBoostMode, Cruise settings). It adds at most one boost to the model's (e2e)
acceleration target, before the planner's min() with the lead MPC and the cruise ramp:

  0 Off               nothing: the model's target passes through unchanged (the default)
  1 (removed)         was Gas, Gas Override Boost (openpilot #39015 via sunnypilot); a stored 1 reads as Off
  2 Sunny Boost       sunnypilot's predictive boost, #2089 (sunnypilot/selfdrive/controls/lib/accel_boost.py,
                      AccelBoostSP; upstream's code, FM's old copy is gone)
  3 EVP Set Speed     EVPilot's Set Speed Assist (set_speed_assist.py, ported on fm-dev-nav-settle)
  4 AP Accel Control  AdventurePilot's Acceleration Control (acceleration_control.py, AP's code)
  5 OP Personality    openpilot #39248 (selfdrive/controls/lib/accel_boost.py, AccelBoost, openpilot's file unchanged);
                      replaced #39430 + #39440 on Nov 27, 2026

Values 0 and 2-3 mean what they meant on fm-dev-nav-settle (same key; there 2 was called Predictive and 3 Set Speed).
The numbers are kept as they are so a stored value never changes meaning; 1 is not a mode any more.

The picker alone decides. sunnypilot's AccelBoostSP toggle and AP's AccelerationControl toggle are hidden from the
settings and never read here: Sunny Boost runs through AccelBoostSP.update() without its update_params() (the toggle
read), and AP Accel Control through AccelerationControl with params that always say on. So no two boosts can run
together. In the base they could: Acceleration Control ran with Gas Override Boost on top of it, and Gas Override Boost
ran whenever Acceleration Boost was off. Gas Override Boost itself is gone from FM Pilot.

Each mode starts clean when it is picked (no boost carries over from the previous one). What the mode added this frame
(`applied`, its output minus its input) is what the planner publishes as longitudinalPlan.accelBoost for every mode,
so the Boost bar reads one field; the mode in use goes in fmPlan.accelBoostMode.
"""
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost as OpAccelBoost, get_accel_boost
from openpilot.selfdrive.controls.lib.curve_speed import CurveSpeed
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.accel_boost import AccelBoostSP, get_boost_max as get_sunny_max
from openpilot.sunnypilot.selfdrive.controls.lib.acceleration_control import AccelerationControl, ACCELERATION_CONTROL_BOOST
from openpilot.sunnypilot.selfdrive.controls.lib.set_speed_assist import SetSpeedAssist, get_max_boost as get_evp_max

PARAM_MODE = "FM_AccelBoostMode"
PARAM_CURVE_HOLD = "FM_OpBoostCurveHold"  # OP Personality: hold the boost off in curves, as openpilot (default on)
MODE_OFF, MODE_SUNNY, MODE_EVP_SET_SPEED, MODE_AP_ACCEL_CONTROL, MODE_OP_PERSONALITY = 0, 2, 3, 4, 5
REMOVED_MODES = (1,)  # 1 was Gas (Gas Override Boost); reads as Off
MODES = (MODE_OFF, MODE_SUNNY, MODE_EVP_SET_SPEED, MODE_AP_ACCEL_CONTROL, MODE_OP_PERSONALITY)
DEFAULT_MODE = MODE_OFF
READ_FRAMES = int(PARAMS_UPDATE_PERIOD / DT_MDL)

Personality = log.LongitudinalPersonality

# OP Personality's Boost bar full scale: its bias plus its gain on a 1 m/s^2 model request (a firm pull-away). openpilot's
# boost has no fixed maximum (the gain scales with the model's request), so the bar tops out there.
OP_BAR_MODEL_ACCEL = 1.0


def personality_index(personality) -> int:
  """log.LongitudinalPersonality value (0 aggressive, 1 standard, 2 relaxed); anything else counts as standard."""
  try:
    idx = int(getattr(personality, 'raw', personality))
  except (TypeError, ValueError):
    return int(Personality.standard)
  return idx if idx in (int(Personality.aggressive), int(Personality.standard), int(Personality.relaxed)) else int(Personality.standard)


def op_boost_max(personality) -> float:
  bias, gain = get_accel_boost(personality_index(personality))
  return bias + gain * OP_BAR_MODEL_ACCEL


class OpPersonalityBoost:
  """Mode 5: openpilot's AccelBoost (#39248) unchanged, fed as openpilot's planner feeds it:
  - curve_offset: openpilot's CurveSpeed (curve_speed.py, unchanged) on the model's plan and vehicleParameters.roll.
    openpilot boosts only while that offset is 0. With Hold Boost in Curves on (FM_OpBoostCurveHold, the default) FM
    does the same; off, the boost also runs in curves. FM never adds the offset to the target (sunnypilot's Smart Cruise
    Control does FM's curve slowing), so this changes only when the boost runs.
  - reset() whenever the planner resets its state (not engaged, or the driver controlling the speed), as openpilot.
  - personality: an unknown value would raise in openpilot's get_accel_boost; here the boost then adds nothing.
  - a plan without times (position.t empty or not matching position.x; real modelV2 always has both) would raise in
    openpilot's plan-ahead read (it checks only position.x); here it adds nothing and starts over."""
  def __init__(self):
    self.op = OpAccelBoost()
    self.curve_speed = CurveSpeed(DT_MDL)
    self.curve_offset = 0.0  # openpilot's CurveSpeed offset this frame (non-zero: openpilot is slowing for a curve)

  def reset(self) -> None:
    self.op.reset()
    self.curve_speed.reset()
    self.curve_offset = 0.0

  def update(self, sm, output_a_target_e2e: float, hold_in_curves: bool = True) -> float:
    model = sm['modelV2']
    pos = model.position
    if len(pos.x) and (len(pos.t) != len(pos.x) or pos.t[-1] <= 0.0):
      self.reset()
      return output_a_target_e2e
    try:
      roll = float(sm['vehicleParameters'].roll)
    except (KeyError, AttributeError):
      roll = 0.0
    self.curve_offset = self.curve_speed.update(model, roll)
    curve_offset = self.curve_offset if hold_in_curves else 0.0
    try:
      return float(self.op.update(sm, output_a_target_e2e, curve_offset))
    except NotImplementedError:
      return output_a_target_e2e


def parse_mode(value) -> int:
  try:
    mode = int(value)
  except (TypeError, ValueError):
    return DEFAULT_MODE
  return mode if mode in MODES else DEFAULT_MODE


def boost_max(mode: int, personality) -> float:
  """The most a mode can add (m/s^2), for the Boost bar's full scale."""
  if mode == MODE_SUNNY:
    try:
      return get_sunny_max(personality)
    except NotImplementedError:
      return get_sunny_max()
  if mode == MODE_EVP_SET_SPEED:
    return get_evp_max(personality)
  if mode == MODE_AP_ACCEL_CONTROL:
    return ACCELERATION_CONTROL_BOOST[personality_index(personality)]
  if mode == MODE_OP_PERSONALITY:
    return op_boost_max(personality)
  return 0.0


class _AlwaysOn:
  """Params for AP's Acceleration Control: in mode 4 the picker has chosen it, so its own toggle is not read."""
  @staticmethod
  def get_bool(key, *args, **kwargs):
    return True


class FmAccelBoostMode:
  def __init__(self, CP, params=None):
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    self.params = params
    self.CP = CP
    self.frame = -1
    self.mode = None
    self.hold_in_curves = True
    self.applied = 0.0  # what the boost added to the model's target this frame (longitudinalPlan.accelBoost)
    self._reset()
    self.read_params()

  def _reset(self) -> None:
    self.sunny = AccelBoostSP()
    self.set_speed = SetSpeedAssist()
    self.ap_accel_control = AccelerationControl(self.CP, params=_AlwaysOn())
    self.op_personality = OpPersonalityBoost()
    self.applied = 0.0

  def read_params(self) -> None:
    hold = self.params.get(PARAM_CURVE_HOLD, return_default=True)
    self.hold_in_curves = True if hold is None else bool(hold)
    mode = parse_mode(self.params.get(PARAM_MODE, return_default=True))
    if mode != self.mode:
      self.mode = mode
      self._reset()

  def planner_reset(self) -> None:
    """The planner reset its state (not engaged, or the driver controlling the speed). openpilot resets its AccelBoost
    and CurveSpeed there; so does OP Personality. The other modes keep their own handling."""
    self.op_personality.reset()

  @property
  def acceleration_control_boost(self) -> float:
    """AP Accel Control's requested lift, for AP's longitudinalPlanSP.accelerationControlBoostRequested (0 in other modes)."""
    return float(self.ap_accel_control.boost) if self.mode == MODE_AP_ACCEL_CONTROL else 0.0

  def update(self, sm, output_a_target_e2e: float, output_a_target_mpc: float, a_cruise: float,
             throttle_scale: float = 1.0) -> float:
    self.frame += 1
    if self.frame % READ_FRAMES == 0:
      self.read_params()

    out = output_a_target_e2e
    if self.mode == MODE_SUNNY:
      out = self.sunny.update(sm, output_a_target_e2e, 0.0)
    elif self.mode == MODE_EVP_SET_SPEED:
      out = self.set_speed.update(sm, output_a_target_e2e, a_cruise, output_a_target_mpc)
    elif self.mode == MODE_AP_ACCEL_CONTROL:
      out = self.ap_accel_control.update(sm, output_a_target_e2e, throttle_scale)
    elif self.mode == MODE_OP_PERSONALITY:
      out = self.op_personality.update(sm, output_a_target_e2e, self.hold_in_curves)
    self.applied = float(out - output_a_target_e2e)
    return out
