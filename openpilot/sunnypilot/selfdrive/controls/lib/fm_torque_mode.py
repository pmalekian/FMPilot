"""
FM Pilot: Headroom, the torque controller's side of the Rivian "Max Torque" setting. On drives that started with Max
Torque above 385 (RivianFlagsSP.HIGH_TORQUE_PANDA), the panda and the car code allow the higher low-speed curve
(CarControllerParams.steer_max_lookup_high: the drive's Max Torque under 20 mph, back to AP's curve by 38 mph), and this
lets the controller ask for it: the most it may request goes from 1.0 to ratio(v) = higher curve / AP's curve. Requests
are not scaled, so everyday steering is unchanged; only when the controller would have saturated at 1.0 can it go
further. At 385, or on other cars, it is always (1.0), AP's behavior.

Headroom is the only mode (folded: fm-dev-nav-settle's FM_TorqueMode Off / Headroom / Full is gone). As there, the
extra blends in over BLEND_S at the start of the drive.
"""
import math
import numpy as np

from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.fm_math import step_toward

BLEND_S = 1.0  # s


def _lookups(cap: int):
  from opendbc.car.rivian.values import CarControllerParams
  return CarControllerParams.STEER_MAX_LOOKUP, CarControllerParams.steer_max_lookup_high(cap)


def _ratio(v_ego: float, base, high) -> float:
  b = round(float(np.interp(v_ego, *base)))
  h = max(b, round(float(np.interp(v_ego, *high))))
  return h / b


def high_torque_ratio(v_ego: float, cap: int) -> float:
  """Higher cap / AP's cap at this speed, rounded as the car code rounds them; never below 1."""
  return _ratio(v_ego, *_lookups(cap))


def drive_torque_cap(CP_SP) -> int:
  """The low-speed limit the panda enforces this drive (Max Torque; 385 = stock)."""
  from opendbc.sunnypilot.car.rivian.values import high_torque_cap
  return high_torque_cap(int(CP_SP.safetyParam))


def high_torque_available(CP, CP_SP) -> bool:
  if CP.brand != 'rivian':
    return False
  from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP
  return bool(CP_SP.flags & RivianFlagsSP.HIGH_TORQUE_PANDA)


class FmTorqueMode:
  def __init__(self, CP, CP_SP):
    self.available = high_torque_available(CP, CP_SP)
    self.cap = drive_torque_cap(CP_SP) if self.available else 385
    self.w_headroom = 0.0  # 0..1, blends in over BLEND_S
    # the two speed curves, looked up once: they depend only on this drive's cap
    self.lookups = _lookups(self.cap) if self.available else None

  def update(self, v_ego: float) -> float:
    """The most the torque controller may request this frame (1.0 = AP's curve)."""
    if not self.available or not math.isfinite(v_ego):
      return 1.0
    self.w_headroom = step_toward(self.w_headroom, 1.0, DT_CTRL / BLEND_S)
    return 1.0 + (_ratio(v_ego, *self.lookups) - 1.0) * self.w_headroom
