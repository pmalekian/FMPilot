"""
FM Pilot: Calmer Low-Speed Correction (Rivian, on by default).

openpilot's torque controllers (v0 and v1) correct the gap between the lateral acceleration asked and the one measured
with a proportional gain that grows steeply at low speed (KP_INTERP: 30 at 3 m/s, 11.5 at 5 m/s, against 1.0 at
highway speed), because lateral acceleration itself is tiny there. With the steering torque only able to build at about
4 counts per 10 ms, that correction runs ahead of the truck: it pushes to the torque cap while the truck is still
catching up, then flips the other way once the gap closes. In Nov 11 logs the request swung across the whole range
mid-turn (00000068--1 +24.3 to +25.5 s: 580 left, then right for a moment, then 580 left again, in a left turn;
00000061--10 +14.2 to +15.6 s: -580, +528, -580). The slow torque ramp smoothed most of it out of the wheel.

This scales that low-speed gain by Calmer Low-Speed Correction Strength (40-100 %, default 70 %) below 7 m/s (about
16 mph), blending back to the normal gain by 9 m/s (about 20 mph); highway steering is unchanged. Changing the setting
blends over a second. Only the proportional gain changes: the feedforward, friction and the integrator (frozen below
5 m/s anyway) are untouched, and every torque limit after the controller still applies. A lower gain also builds less
torque early in a turn, which is what Turn Lead and Smooth Turn Start cover.
"""
import math

import numpy as np

from openpilot.common.realtime import DT_CTRL

PARAM_ENABLED = "FM_LowSpeedGainEnabled"
PARAM_PERCENT = "FM_LowSpeedGainPercent"
PERCENT_LIMITS = (40.0, 100.0)
PERCENT_DEFAULT = 70.0
SPEEDS = (7.0, 9.0)  # m/s: the set strength below the first, normal gain from the second
BLEND_S = 1.0  # s, from one strength to another
PARAMS_PERIOD = 1.0  # s


def scaled_kp_table(speeds, kps, scale: float) -> tuple[list[float], list[float]]:
  """The controller's kp table with the low-speed scale applied; SPEEDS are added as breakpoints so the blend is exact."""
  bp = sorted({float(s) for s in speeds} | set(SPEEDS))
  return bp, [float(np.interp(s, speeds, kps)) * float(np.interp(s, SPEEDS, [scale, 1.0])) for s in bp]


class FmLowSpeedGain:
  def __init__(self, CP, params=None):
    self.available = getattr(CP, "brand", None) == "rivian"
    self.params = params
    self.target = 1.0
    self.scale = 1.0
    self.frame = -1
    if self.available:
      if self.params is None:
        from openpilot.common.params import Params
        self.params = Params()
      self.read_params()
      self.scale = self.target  # the drive starts at the setting, no blend

  def read_params(self) -> None:
    try:
      enabled = self.params.get_bool(PARAM_ENABLED)
      percent = float(self.params.get(PARAM_PERCENT, return_default=True) or PERCENT_DEFAULT)
    except (TypeError, ValueError):
      enabled, percent = False, 100.0
    if not math.isfinite(percent):
      percent = PERCENT_DEFAULT
    self.target = float(np.clip(percent, *PERCENT_LIMITS)) / 100.0 if enabled else 1.0

  def update(self) -> float:
    """The low-speed gain scale for this frame (1.0 = openpilot's gain)."""
    if not self.available:
      return 1.0
    self.frame += 1
    if self.frame % int(PARAMS_PERIOD / DT_CTRL) == 0:
      self.read_params()
    step = DT_CTRL / BLEND_S
    self.scale = float(np.clip(self.target, self.scale - step, self.scale + step))
    return self.scale

  def apply(self, lac, speeds, kps) -> None:
    """Point the controller's PID at the scaled table when the scale changed."""
    scale = self.update()
    if scale != getattr(lac, "_fm_kp_scale", 1.0):
      lac.pid._k_p = scaled_kp_table(speeds, kps, scale)
      lac._fm_kp_scale = scale
