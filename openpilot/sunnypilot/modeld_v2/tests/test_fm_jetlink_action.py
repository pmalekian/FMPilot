import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.modeld_v2.fm_jetlink_action import with_blendful


def _stock(model_output, prev_action, lat_action_t, long_action_t, v_ego):
  return log.ModelDataV2.Action(desiredCurvature=0.01, desiredAcceleration=0.5)


def test_takes_modeld_v2s_sixth_argument_and_adds_blendful():
  fn = with_blendful(_stock, 0.3)
  seen = []
  out = fn({}, None, 0.0, 0.0, 10.0, lambda c: seen.append(c) or 0.002)
  assert seen == [pytest.approx(0.01)]
  assert out.desiredCurvature == pytest.approx(0.012) and out.desiredAcceleration == 0.5


def test_without_blendful_or_below_min_speed_is_stock():
  fn = with_blendful(_stock, 0.3)
  stock = _stock({}, None, 0.0, 0.0, 10.0).desiredCurvature
  assert fn({}, None, 0.0, 0.0, 10.0).desiredCurvature == stock
  assert fn({}, None, 0.0, 0.0, 10.0, lambda c: 0.0).desiredCurvature == stock
  assert fn({}, None, 0.0, 0.0, 0.2, lambda c: 0.002).desiredCurvature == stock
