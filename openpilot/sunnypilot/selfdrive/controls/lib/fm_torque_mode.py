"""
FM Pilot: Higher Torque, a Rivian setting that lets the torque controller use more than AP's steering torque cap at low
speed. Works only on drives that started with "Panda Allows Higher Torque" on (RivianFlagsSP.HIGH_TORQUE_PANDA); the
panda and the car code then allow the higher curve (CarControllerParams.STEER_MAX_LOOKUP_HIGH: 481 under 20 mph, back to
AP's curve by 38 mph). Otherwise every mode behaves as Off.

openpilot's torque request is a fraction of AP's curve (1.0 = 385 at low speed). ratio(v) = high curve / AP's curve.

  Off (0):      max request 1.0, unscaled. Exactly AP's behavior.
  Headroom (1): max request ratio(v), unscaled. Everyday steering is unchanged; only when the controller would have
                saturated at 1.0 can it go further, up to the higher cap.
  Full (2):     max request 1.0, then scaled by ratio(v). Every request at low speed is stronger, as AP's 481 test
                (5270cbba) was. For comparison with Headroom.

Changing the mode blends over BLEND_S, so switching mid-drive or mid-turn never steps the torque.
"""
import numpy as np

from openpilot.common.realtime import DT_CTRL

PARAM_MODE = "FM_TorqueMode"
OFF, HEADROOM, FULL = 0, 1, 2
BLEND_S = 1.0       # s, from one mode to another
PARAMS_PERIOD = 1.0  # s


def _lookups():
  from opendbc.car.rivian.values import CarControllerParams
  return CarControllerParams.STEER_MAX_LOOKUP, CarControllerParams.STEER_MAX_LOOKUP_HIGH


def high_torque_ratio(v_ego: float) -> float:
  """Higher cap / AP's cap at this speed, rounded as the car code rounds them; never below 1."""
  base, high = _lookups()
  b = round(float(np.interp(v_ego, *base)))
  h = max(b, round(float(np.interp(v_ego, *high))))
  return h / b


def high_torque_available(CP, CP_SP) -> bool:
  if CP.brand != 'rivian':
    return False
  from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP
  return bool(CP_SP.flags & RivianFlagsSP.HIGH_TORQUE_PANDA)


class FmTorqueMode:
  def __init__(self, CP, CP_SP, params=None):
    self.available = high_torque_available(CP, CP_SP)
    self.params = params
    self.mode = OFF
    self.w_headroom = 0.0  # 0..1, blended toward the selected mode
    self.w_full = 0.0
    self.frame = -1
    if self.available:
      if self.params is None:
        from openpilot.common.params import Params
        self.params = Params()
      self.read_params()

  def read_params(self):
    try:
      mode = int(self.params.get(PARAM_MODE, return_default=True) or 0)
    except Exception:
      mode = OFF
    self.mode = mode if mode in (OFF, HEADROOM, FULL) else OFF

  def update(self, v_ego: float) -> tuple[float, float]:
    """Returns (max request, request scale) for this frame. (1.0, 1.0) is AP's behavior."""
    if not self.available:
      return 1.0, 1.0
    self.frame += 1
    if self.frame % int(PARAMS_PERIOD / DT_CTRL) == 0:
      self.read_params()

    step = DT_CTRL / BLEND_S
    target_h = 1.0 if self.mode == HEADROOM else 0.0
    target_f = 1.0 if self.mode == FULL else 0.0
    self.w_headroom = float(np.clip(target_h, self.w_headroom - step, self.w_headroom + step))
    self.w_full = float(np.clip(target_f, self.w_full - step, self.w_full + step))

    extra = high_torque_ratio(v_ego) - 1.0
    return 1.0 + extra * self.w_headroom, 1.0 + extra * self.w_full
