"""FM Pilot: FM_LateralSmoothingEnabled replaces the model bundle's `lat` value (LAT_SMOOTH_SECONDS) in modeld_v2."""
import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.modeld_v2.modeld import ModelState, fm_lat_smooth_seconds


class FakeParams:
  def __init__(self, enabled=False, seconds=0.1):
    self.values = {"FM_LateralSmoothingEnabled": enabled, "FM_LateralSmoothingSeconds": seconds}

  def get_bool(self, key):
    assert key == "FM_LateralSmoothingEnabled"
    return self.values[key]

  def get(self, key, return_default=False):
    assert key == "FM_LateralSmoothingSeconds" and return_default
    return self.values[key]


@pytest.mark.parametrize("bundle", [0.0, 0.1, 0.3])
@pytest.mark.parametrize("seconds", [0.0, 0.1, 0.2])
def test_off_keeps_the_bundle_value(bundle, seconds):
  assert fm_lat_smooth_seconds(FakeParams(False, seconds), bundle) == bundle


@pytest.mark.parametrize("bundle", [0.0, 0.1, 0.3])
@pytest.mark.parametrize("seconds", [0.0, 0.1, 0.2])
def test_on_replaces_the_bundle_value(bundle, seconds):
  assert fm_lat_smooth_seconds(FakeParams(True, seconds), bundle) == seconds


@pytest.mark.parametrize("seconds,expected", [(-1.0, 0.0), (0.5, 0.2), (0.2, 0.2), (1e9, 0.2)])
def test_out_of_range_is_clamped(seconds, expected):
  assert fm_lat_smooth_seconds(FakeParams(True, seconds), 0.1) == expected


@pytest.mark.parametrize("seconds", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_falls_back_to_the_bundle_value(seconds):
  assert fm_lat_smooth_seconds(FakeParams(True, seconds), 0.1) == 0.1


def _step_curvature(smooth_seconds, steps=1):
  """Curvature a model that jumps from 0 to a constant output asks for after `steps` frames."""
  class S:
    pass
  state = S()
  state.__dict__.update(PLANPLUS_CONTROL=1.0, LONG_SMOOTH_SECONDS=0.0, LAT_SMOOTH_SECONDS=smooth_seconds,
                        MIN_LAT_CONTROL_SPEED=0.3, mlsim=True, generation=12)
  out = {'action': np.array([[1.0, 0.0]], dtype=np.float32)}
  prev = log.ModelDataV2.Action()
  for _ in range(steps):
    prev = ModelState.get_action_from_model(state, out, prev, 0.0, 0.0, 10.0)
  return prev.desiredCurvature


def test_zero_passes_the_model_through_unsmoothed():
  assert _step_curvature(0.0) == pytest.approx(1.0 / 10.0 ** 2)


def test_more_seconds_smooths_more():
  zero, one, two = (_step_curvature(s) for s in (0.0, 0.1, 0.2))
  assert zero > one > two > 0.0
  assert one == pytest.approx((1 - np.exp(-DT_MDL / 0.1)) * zero, rel=1e-4)
  assert two == pytest.approx((1 - np.exp(-DT_MDL / 0.2)) * zero, rel=1e-4)
