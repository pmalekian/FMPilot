# FM Pilot: rear-axle off-tracking compensation from gerrylum's rx-wb fork (commit ce7e5c8), MIT license.
import math


def offtrack_curvature(curvature: float, wheelbase: float, gain: float) -> float:
  # In a turn the rear axle cuts inside the front's path by r - sqrt(r^2 - L^2). Widen the radius by gain * that cut-in.
  # Worked in curvature (k = 1/r) so a near-zero curvature can't blow up: r - sqrt(r^2 - L^2) = r * c with
  # c = 1 - sqrt(1 - (L*k)^2), written as (L*k)^2 / (1 + sqrt(1 - (L*k)^2)) to stay exact for small k.
  # The old radius form squared r and raised OverflowError (crashing controlsd) for |k| below ~1e-154.
  x = wheelbase * curvature
  if gain <= 0. or not math.isfinite(curvature) or abs(x) >= 1.:
    return curvature
  cut = x * x / (1. + math.sqrt(1. - x * x))
  return curvature / (1. + gain * cut)
