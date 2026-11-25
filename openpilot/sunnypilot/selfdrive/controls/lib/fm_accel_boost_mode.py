"""
FM Pilot: the Accel Boost picker (FM_AccelBoostMode, Cruise settings). It adds at most one boost to the model's (e2e)
acceleration target, before the planner's min() with the lead MPC and the cruise ramp:

  0 Off               nothing: the model's target passes through unchanged (the default)
  1 Gas               Gas Override Boost, openpilot #39015 (Harald Schäfer) via sunnypilot
                      (selfdrive/controls/lib/accel_boost.py, AccelBoost)
  2 Sunny Boost       sunnypilot's predictive boost, #2089 (sunnypilot/selfdrive/controls/lib/accel_boost.py,
                      AccelBoostSP; upstream's code, FM's old copy is gone)
  3 EVP Set Speed     EVPilot's Set Speed Assist (set_speed_assist.py, ported on fm-dev-nav-settle)
  4 AP Accel Control  AdventurePilot's Acceleration Control (acceleration_control.py, AP's code)
  5 OP Personality    openpilot #39430 + #39440 (fm_op_personality_boost.py)

Values 0-3 mean what they meant on fm-dev-nav-settle (same key; there 2 was called Predictive and 3 Set Speed).

The picker alone decides. sunnypilot's AccelBoostSP toggle and AP's AccelerationControl toggle are hidden from the
settings and never read here: Sunny Boost runs through AccelBoostSP.update() without its update_params() (the toggle
read), and AP Accel Control through AccelerationControl with params that always say on. So no two boosts can run
together. In the base they could: Acceleration Control ran with Gas Override Boost on top of it, and Gas Override Boost
ran whenever Acceleration Boost was off.

Each mode starts clean when it is picked (no boost carries over from the previous one). What the mode added this frame
(`applied`, its output minus its input) is what the planner publishes as longitudinalPlan.accelBoost for every mode,
so the Boost bar reads one field; the mode in use goes in fmPlan.accelBoostMode.
"""
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost, ACCEL_BOOST_MAX as GAS_BOOST_MAX
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.accel_boost import AccelBoostSP, get_boost_max as get_sunny_max
from openpilot.sunnypilot.selfdrive.controls.lib.acceleration_control import AccelerationControl, ACCELERATION_CONTROL_BOOST
from openpilot.sunnypilot.selfdrive.controls.lib.fm_op_personality_boost import OpPersonalityBoost, personality_index
from openpilot.sunnypilot.selfdrive.controls.lib.fm_op_personality_boost import get_max_boost as get_op_max
from openpilot.sunnypilot.selfdrive.controls.lib.set_speed_assist import SetSpeedAssist, get_max_boost as get_evp_max

PARAM_MODE = "FM_AccelBoostMode"
MODE_OFF, MODE_GAS, MODE_SUNNY, MODE_EVP_SET_SPEED, MODE_AP_ACCEL_CONTROL, MODE_OP_PERSONALITY = range(6)
MODES = (MODE_OFF, MODE_GAS, MODE_SUNNY, MODE_EVP_SET_SPEED, MODE_AP_ACCEL_CONTROL, MODE_OP_PERSONALITY)
DEFAULT_MODE = MODE_OFF
READ_FRAMES = int(PARAMS_UPDATE_PERIOD / DT_MDL)


def parse_mode(value) -> int:
  try:
    mode = int(value)
  except (TypeError, ValueError):
    return DEFAULT_MODE
  return mode if mode in MODES else DEFAULT_MODE


def boost_max(mode: int, personality) -> float:
  """The most a mode can add (m/s^2), for the Boost bar's full scale."""
  if mode == MODE_GAS:
    return GAS_BOOST_MAX
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
    return get_op_max(personality)
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
    self.applied = 0.0  # what the boost added to the model's target this frame (longitudinalPlan.accelBoost)
    self._reset()
    self.read_params()

  def _reset(self) -> None:
    self.gas = AccelBoost()
    self.sunny = AccelBoostSP()
    self.set_speed = SetSpeedAssist()
    self.ap_accel_control = AccelerationControl(self.CP, params=_AlwaysOn())
    self.op_personality = OpPersonalityBoost()
    self.applied = 0.0

  def read_params(self) -> None:
    mode = parse_mode(self.params.get(PARAM_MODE, return_default=True))
    if mode != self.mode:
      self.mode = mode
      self._reset()

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
    if self.mode == MODE_GAS:
      out = self.gas.update(sm, output_a_target_e2e, output_a_target_mpc, a_cruise)
    elif self.mode == MODE_SUNNY:
      out = self.sunny.update(sm, output_a_target_e2e, output_a_target_mpc, a_cruise)
    elif self.mode == MODE_EVP_SET_SPEED:
      out = self.set_speed.update(sm, output_a_target_e2e, a_cruise, output_a_target_mpc)
    elif self.mode == MODE_AP_ACCEL_CONTROL:
      out = self.ap_accel_control.update(sm, output_a_target_e2e, throttle_scale)
    elif self.mode == MODE_OP_PERSONALITY:
      out = self.op_personality.update(sm, output_a_target_e2e)
    self.applied = float(out - output_a_target_e2e)
    return out
