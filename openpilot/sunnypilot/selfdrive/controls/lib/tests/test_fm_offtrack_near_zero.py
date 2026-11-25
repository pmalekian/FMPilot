"""FM Pilot: AP's rear off-tracking works in curvature, so a near-zero curvature can't raise OverflowError in controlsd
(EVPilot a0500916's crash fix; these tests are FM Pilot's, from fm-dev-nav-settle)."""
import math
import random

import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.rivian_offtrack import offtrack_curvature

WHEELBASE = 3.08  # R1S


def radius_form(curvature, wheelbase, gain):
  """AP's radius-based formula before the fix, as reference where it doesn't overflow."""
  radius = 1. / abs(curvature) if curvature else math.inf
  if gain <= 0. or not wheelbase < radius < math.inf:
    return curvature
  return math.copysign(1. / (radius + gain * (radius - math.sqrt(radius ** 2 - wheelbase ** 2))), curvature)


@pytest.mark.parametrize("radius", [3.1, 4., 5., 10., 30., 100., 1000., 1e6])
@pytest.mark.parametrize("gain", [0.1, 0.5, 1.0])
@pytest.mark.parametrize("sign", [1., -1.])
def test_matches_the_radius_form(radius, gain, sign):
  k = sign / radius
  assert offtrack_curvature(k, WHEELBASE, gain) == pytest.approx(radius_form(k, WHEELBASE, gain), rel=1e-12)


def test_same_as_the_radius_form_on_random_inputs():
  rng = random.Random(0)
  for _ in range(20000):
    k = rng.choice((1., -1.)) * 10 ** rng.uniform(-12, 0.5)
    gain, wheelbase = rng.choice((0., rng.uniform(0., 1.))), rng.choice((3.08, 3.45))
    assert offtrack_curvature(k, wheelbase, gain) == pytest.approx(radius_form(k, wheelbase, gain), rel=1e-9, abs=1e-15)


@pytest.mark.parametrize("k", [0., 1e-9, 1e-100, 1e-154, 1e-200, 1e-300, -1e-200, 5e-324])
def test_near_zero_curvature_does_not_overflow(k):
  # lane guard's low-pass used to leave ~1e-200: the radius form squared 1e200 and raised OverflowError, killing controlsd
  out = offtrack_curvature(k, WHEELBASE, 0.5)
  assert out == pytest.approx(k, abs=1e-300) and math.copysign(1., out) == math.copysign(1., k)


def test_the_old_form_did_overflow():
  with pytest.raises(OverflowError):
    radius_form(1e-200, WHEELBASE, 0.5)


@pytest.mark.parametrize("k", [math.inf, -math.inf, math.nan])
def test_non_finite_passes_through(k):
  out = offtrack_curvature(k, WHEELBASE, 0.5)
  assert (math.isnan(out) and math.isnan(k)) or out == k


@pytest.mark.parametrize("k", [1 / 3.08, 1 / 2., 1 / 1.])
def test_a_turn_tighter_than_the_wheelbase_is_left_alone(k):
  assert offtrack_curvature(k, WHEELBASE, 0.5) == k
