# FM Pilot: Accel Boost mode. One of No Boost, Gas Boost (Gas Override Boost, accel_boost.py) or Predictive Boost
# (sunnypilot AccelBoostSP, sunnypilot/selfdrive/controls/lib/accel_boost.py). Exclusive, so the two boosts never stack.
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.accel_boost import AccelBoostSP

PARAM_MODE = "FM_AccelBoostMode"
MODE_OFF, MODE_GAS, MODE_PREDICTIVE = 0, 1, 2
DEFAULT_MODE = MODE_GAS  # matches FM Pilot before the selector: Gas Boost always on


def parse_mode(value) -> int:
  try:
    mode = int(value)
  except (TypeError, ValueError):
    return DEFAULT_MODE
  return mode if mode in (MODE_OFF, MODE_GAS, MODE_PREDICTIVE) else DEFAULT_MODE


class FmBoostMode:
  def __init__(self, params):
    self.params = params
    self.frame = -1
    self.mode = None
    self.gas = AccelBoost()
    self.predictive = AccelBoostSP()
    self.read_params()

  def read_params(self) -> None:
    mode = parse_mode(self.params.get(PARAM_MODE, return_default=True))
    if mode != self.mode:
      # Start each mode clean: no boost carries over from the previous one.
      self.mode = mode
      self.gas = AccelBoost()
      self.predictive.reset()

  @property
  def total_boost(self) -> float:
    if self.mode == MODE_GAS:
      return self.gas.total_boost
    if self.mode == MODE_PREDICTIVE:
      return self.predictive.total_boost
    return 0.0

  def update(self, sm, output_a_target_e2e, output_a_target_mpc, a_cruise):
    self.frame += 1
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.read_params()
    if self.mode == MODE_GAS:
      return self.gas.update(sm, output_a_target_e2e, output_a_target_mpc, a_cruise)
    if self.mode == MODE_PREDICTIVE:
      return self.predictive.update(sm, output_a_target_e2e, output_a_target_mpc, a_cruise)
    return output_a_target_e2e
