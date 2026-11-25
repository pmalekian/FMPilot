"""
Rear-axle off-tracking compensation for the Rivian R1T and R1S, ported from Gerrylum's rx-wb-stg (ce7e5c82cf).

In a turn the rear axle cuts inside the front's path by r - sqrt(r^2 - L^2). This widens openpilot's turn by
gain times that cut-in, so the rear wheels clear the curb the front wheels cleared. A geometric estimate from
the wheelbase, not curb sensing. Off by default.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import numpy as np


def offtrack_curvature(curvature: float, wheelbase: float, gain: float) -> float:
  # FM Pilot: worked in curvature (k = 1/r) so a near-zero curvature can't blow up: r - sqrt(r^2 - L^2) = r * c with
  # c = 1 - sqrt(1 - (L*k)^2), written as (L*k)^2 / (1 + sqrt(1 - (L*k)^2)) to stay exact for small k. The radius form
  # squared r and raised OverflowError (crashing controlsd) for |k| below ~1e-154. Same result elsewhere: unchanged at
  # 0, NaN, inf and |L*k| >= 1 (the radius form's r <= L). Near-zero curvature crash fix from EVPilot a0500916
  # (Gerry / gerrylum, MIT), as carried on FM Pilot fm-dev-nav-settle.
  x = wheelbase * curvature
  if gain <= 0. or not math.isfinite(curvature) or abs(x) >= 1.:
    return curvature
  cut = x * x / (1. + math.sqrt(1. - x * x))
  return curvature / (1. + gain * cut)


class RivianOfftrack:
  def __init__(self, CP, params):
    # Takes the caller's Params, as lane centering does, so this stays testable off-device
    self.params = params
    self.wheelbase = CP.wheelbase  # the VIN-aware figure: 3.45 m on an R1T, 3.08 m otherwise
    self.is_rivian = CP.brand == 'rivian'
    self.gain = 0.0

  def get_params(self) -> None:
    if not self.is_rivian or not self.params.get_bool("RivianOfftrackCompensationEnabled"):
      self.gain = 0.0
      return
    gain = self.params.get("RivianOfftrackCompensationGain", return_default=True)
    self.gain = float(np.clip(gain, 0.0, 1.0))

  def update(self, desired_curvature: float, lat_active: bool) -> float:
    if not lat_active:
      return desired_curvature
    return offtrack_curvature(desired_curvature, self.wheelbase, self.gain)
