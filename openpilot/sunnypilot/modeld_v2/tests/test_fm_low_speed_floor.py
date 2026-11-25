"""FM Pilot: FM_LowSpeedCurvatureFloor (sunnypilot #2091) in modeld_v2's get_action_from_model."""
import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.modeld_v2.modeld import (ACTION_SPEED_FLOOR, ACTION_SPEED_FLOOR_LOW_SPEED_FIX, ModelState,
                                                   fm_action_speed_floor)


class S:
  def __init__(self, **kw):
    self.__dict__.update(kw)


def _curvature(v_ego, floor=None, lat_accel=0.5):
  state = S(PLANPLUS_CONTROL=1.0, LONG_SMOOTH_SECONDS=0.0, LAT_SMOOTH_SECONDS=0.0, MIN_LAT_CONTROL_SPEED=0.3, mlsim=True,
            generation=12)
  if floor is not None:
    state.action_speed_floor = floor
  out = {'action': np.array([[lat_accel, 0.0]], dtype=np.float32)}
  return ModelState.get_action_from_model(state, out, log.ModelDataV2.Action(), 0.0, 0.0, v_ego).desiredCurvature


@pytest.mark.parametrize("v_ego", [0.5, 1.0, 2.0, 2.5, 5.0, 20.0])
def test_default_is_unchanged(v_ego):
  assert _curvature(v_ego) == pytest.approx(0.5 / max(1.0, v_ego) ** 2)
  assert _curvature(v_ego, ACTION_SPEED_FLOOR) == _curvature(v_ego)


def test_floor_lowers_only_crawl_speed_requests():
  fix = ACTION_SPEED_FLOOR_LOW_SPEED_FIX
  assert _curvature(2.0, fix) == pytest.approx(_curvature(2.0) * 0.64)
  assert _curvature(1.0, fix) == pytest.approx(_curvature(1.0) * 0.16)
  for v_ego in (2.5, 5.0, 20.0):
    assert _curvature(v_ego, fix) == _curvature(v_ego)


def test_setting_picks_the_floor():
  class P:
    def __init__(self, on):
      self.on = on

    def get_bool(self, key):
      assert key == "FM_LowSpeedCurvatureFloor"
      return self.on
  assert fm_action_speed_floor(P(False)) == 1.0
  assert fm_action_speed_floor(P(True)) == 2.5
