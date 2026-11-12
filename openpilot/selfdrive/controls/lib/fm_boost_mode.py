# FM Pilot: Accel Boost mode. One of No Boost, Gas Boost (Gas Override Boost, accel_boost.py), Predictive Boost
# (sunnypilot AccelBoostSP, sunnypilot/selfdrive/controls/lib/accel_boost.py) or Set Speed Assist (from EVPilot,
# sunnypilot/selfdrive/controls/lib/set_speed_assist.py). Exclusive, so the boosts never stack.
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost, ACCEL_BOOST_MAX as GAS_BOOST_MAX
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.accel_boost import AccelBoostSP, get_boost_max
from openpilot.sunnypilot.selfdrive.controls.lib.set_speed_assist import SetSpeedAssist, get_max_boost as get_ssa_max

PARAM_MODE = "FM_AccelBoostMode"
MODE_OFF, MODE_GAS, MODE_PREDICTIVE, MODE_SET_SPEED = 0, 1, 2, 3
DEFAULT_MODE = MODE_GAS  # matches FM Pilot before the selector: Gas Boost always on


def parse_mode(value) -> int:
  try:
    mode = int(value)
  except (TypeError, ValueError):
    return DEFAULT_MODE
  return mode if mode in (MODE_OFF, MODE_GAS, MODE_PREDICTIVE, MODE_SET_SPEED) else DEFAULT_MODE


def boost_max(mode: int, personality) -> float:
  """The most a mode can add, for the on-screen boost bar: Gas +0.2, Predictive and Set Speed by personality, Off 0."""
  if mode == MODE_GAS:
    return GAS_BOOST_MAX
  if mode == MODE_PREDICTIVE:
    try:
      return get_boost_max(personality)
    except NotImplementedError:
      return get_boost_max()
  if mode == MODE_SET_SPEED:
    return get_ssa_max(personality)
  return 0.0


class FmBoostMode:
  def __init__(self, params):
    self.params = params
    self.frame = -1
    self.mode = None
    self.gas = AccelBoost()
    self.predictive = AccelBoostSP()
    self.set_speed = SetSpeedAssist()
    self.applied = 0.0  # what the boost actually added to the model's target this frame (logged, and shown on screen)
    self.read_params()

  def read_params(self) -> None:
    mode = parse_mode(self.params.get(PARAM_MODE, return_default=True))
    if mode != self.mode:
      # Start each mode clean: no boost carries over from the previous one.
      self.mode = mode
      self.gas = AccelBoost()
      self.predictive.reset()
      self.set_speed = SetSpeedAssist()

  @property
  def total_boost(self) -> float:
    if self.mode == MODE_GAS:
      return self.gas.total_boost
    if self.mode == MODE_PREDICTIVE:
      return self.predictive.total_boost
    if self.mode == MODE_SET_SPEED:
      return self.set_speed.total_boost
    return 0.0

  def update(self, sm, output_a_target_e2e, output_a_target_mpc, a_cruise):
    self.frame += 1
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.read_params()
    out = output_a_target_e2e
    if self.mode == MODE_GAS:
      out = self.gas.update(sm, output_a_target_e2e, output_a_target_mpc, a_cruise)
    elif self.mode == MODE_PREDICTIVE:
      out = self.predictive.update(sm, output_a_target_e2e, output_a_target_mpc, a_cruise)
    elif self.mode == MODE_SET_SPEED:
      out = self.set_speed.update(sm, output_a_target_e2e, a_cruise, output_a_target_mpc)
    self.applied = float(out - output_a_target_e2e)
    return out
