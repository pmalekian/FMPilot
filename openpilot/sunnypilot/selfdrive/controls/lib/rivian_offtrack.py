import math


def offtrack_curvature(curvature: float, wheelbase: float, gain: float) -> float:
  # In a turn the rear axle cuts inside the front's path by r - sqrt(r^2 - L^2). Widen the radius by gain * that cut-in.
  radius = 1. / abs(curvature) if curvature else math.inf
  if gain <= 0. or not wheelbase < radius < math.inf:
    return curvature
  return math.copysign(1. / (radius + gain * (radius - math.sqrt(radius ** 2 - wheelbase ** 2))), curvature)
