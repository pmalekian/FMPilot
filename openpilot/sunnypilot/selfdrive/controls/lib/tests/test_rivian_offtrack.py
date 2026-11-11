import math

import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.rivian_offtrack import offtrack_curvature

WHEELBASE = 3.08  # R1S


def radius_form(curvature, wheelbase, gain):
  # the original radius-based formula, to check the curvature form against
  radius = 1. / abs(curvature)
  return math.copysign(1. / (radius + gain * (radius - math.sqrt(radius ** 2 - wheelbase ** 2))), curvature)


@pytest.mark.parametrize("radius", [4., 5., 10., 30., 100., 1000.])
@pytest.mark.parametrize("gain", [0.1, 0.5, 1.0])
@pytest.mark.parametrize("sign", [1., -1.])
def test_matches_the_radius_form(radius, gain, sign):
  k = sign / radius
  assert offtrack_curvature(k, WHEELBASE, gain) == pytest.approx(radius_form(k, WHEELBASE, gain), rel=1e-12)


@pytest.mark.parametrize("k", [0., 1e-9, 1e-100, 1e-154, 1e-200, 1e-300, -1e-200, 5e-324])
def test_near_zero_curvature_does_not_overflow(k):
  # lane guard's low-pass used to leave ~1e-200: the radius form squared 1e200 and raised OverflowError, killing controlsd
  out = offtrack_curvature(k, WHEELBASE, 0.5)
  assert out == pytest.approx(k, abs=1e-300) and math.copysign(1., out) == math.copysign(1., k)


@pytest.mark.parametrize("k", [math.inf, -math.inf, math.nan])
def test_non_finite_passes_through(k):
  out = offtrack_curvature(k, WHEELBASE, 0.5)
  assert (math.isnan(out) and math.isnan(k)) or out == k


@pytest.mark.parametrize("k", [1 / 3.08, 1 / 2., 1 / 1.])
def test_a_turn_tighter_than_the_wheelbase_is_left_alone(k):
  assert offtrack_curvature(k, WHEELBASE, 0.5) == k


def test_gain_zero_is_off_and_gain_widens_the_turn():
  k = 1 / 10.
  assert offtrack_curvature(k, WHEELBASE, 0.) == k
  assert 0. < offtrack_curvature(k, WHEELBASE, 0.5) < k
  assert offtrack_curvature(k, WHEELBASE, 1.) < offtrack_curvature(k, WHEELBASE, 0.5)
