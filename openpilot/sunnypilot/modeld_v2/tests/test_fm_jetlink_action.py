from types import SimpleNamespace

import pytest

from openpilot.sunnypilot.modeld_v2.fm_jetlink_action import with_blendful


def _stock(model_output, prev_action, lat_action_t, long_action_t, v_ego, *, action_speed_floor):
  return SimpleNamespace(desiredCurvature=0.01 / action_speed_floor**2, desiredAcceleration=0.5)


def test_takes_modeld_v2s_sixth_argument_and_adds_blendful():
  fn = with_blendful(_stock, 0.3)
  seen = []
  out = fn({}, None, 0.0, 0.0, 10.0, lambda c: seen.append(c) or 0.002)
  assert seen == [pytest.approx(0.01)]
  assert out.desiredCurvature == pytest.approx(0.012) and out.desiredAcceleration == 0.5


def test_without_blendful_or_below_min_speed_is_stock():
  fn = with_blendful(_stock, 0.3)
  stock = _stock({}, None, 0.0, 0.0, 10.0, action_speed_floor=1.0).desiredCurvature
  assert fn({}, None, 0.0, 0.0, 10.0).desiredCurvature == stock
  assert fn({}, None, 0.0, 0.0, 10.0, lambda c: 0.0).desiredCurvature == stock
  assert fn({}, None, 0.0, 0.0, 0.2, lambda c: 0.002).desiredCurvature == stock


def test_preserves_the_crawl_floor_and_updates_lane_keeping_at_the_cutoff():
  fn = with_blendful(_stock, 0.3)
  seen = []
  out = fn({}, None, 0.0, 0.0, 0.3, lambda c: seen.append(c) or 0.002, action_speed_floor=2.5)
  assert seen == [pytest.approx(0.01 / 2.5**2)]
  assert out.desiredCurvature == pytest.approx(seen[0])
