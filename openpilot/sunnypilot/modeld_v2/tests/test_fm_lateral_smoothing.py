"""FM Pilot: ported from fm-dev-nav-settle and zoompilot's BigModelSmoothing tests (6cb59d3, MIT)."""
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value
from openpilot.selfdrive.modeld import modeld as stock_modeld
from openpilot.sunnypilot.modeld_v2.modeld import ModelState
from openpilot.sunnypilot.modeld_v2.fm_lateral_smoothing import (
  PARAM_BIG_MODEL, PARAM_SMALL_MODEL, smooth_big_model_action, smoothing_override,
)


def model_state(bundle_seconds, big=False):
  model = ModelState.__new__(ModelState)
  model.bundle_lat_smooth_seconds = model.LAT_SMOOTH_SECONDS = bundle_seconds
  model.big_model_smoothing = None
  model.generation = 10
  model.MIN_LAT_CONTROL_SPEED = 0.3
  model.LONG_SMOOTH_SECONDS = 0.0
  model.chestnut = big
  return model


class TestSmoothingOptions(OpenpilotTestCase):
  def test_small_model_uses_bundle_or_override(self):
    params = Params()
    model = model_state(0.2)
    for mode, seconds in ((None, 0.2), (1, 0.0), (2, 0.1), (0, 0.2), (99, 0.2)):
      if mode is not None:
        params.put(PARAM_SMALL_MODEL, mode, block=True)
      model.apply_lateral_smoothing(params)
      assert model.LAT_SMOOTH_SECONDS == seconds

  def test_big_model_overrides_filter_without_changing_bundle_delay(self):
    params = Params()
    model = model_state(0.2, big=True)
    output = {"action": np.array([[0.02, 0.5]])}
    prev = log.ModelDataV2.Action(desiredCurvature=0.007)
    for mode, seconds in ((None, 0.2), (1, 0.0), (2, 0.1)):
      if mode is not None:
        params.put(PARAM_BIG_MODEL, mode, block=True)
      model.big_model_smoothing = smoothing_override(params, PARAM_BIG_MODEL)
      action = model.get_action_from_model(output, prev, 0.3, 0.3, 1.0)
      assert action.desiredCurvature == pytest.approx(smooth_value(0.02, prev.desiredCurvature, seconds))
      assert model.LAT_SMOOTH_SECONDS == 0.2


@pytest.mark.parametrize("big,seconds", [(False, 0.1), (True, None), (True, 0.0), (True, 0.1)])
def test_stock_and_jetlink_action_filter(big, seconds):
  model = SimpleNamespace(chestnut=big, LAT_SMOOTH_SECONDS=stock_modeld.LAT_SMOOTH_SECONDS)
  previous = log.ModelDataV2.Action(desiredCurvature=0.003)
  for raw in np.linspace(-0.03, 0.03, 25):
    output = {"action": np.array([[raw, 0.5]])}
    action = stock_modeld.get_action_from_model(output, previous, 0.3, 0.3, 1.0)
    accel = action.desiredAcceleration
    raw_curvature = action.desiredCurvature
    smooth_big_model_action(action, previous, model, seconds)
    expected = smooth_value(raw_curvature, previous.desiredCurvature, seconds) if big and seconds else raw_curvature
    assert action.desiredCurvature == pytest.approx(expected)
    assert action.desiredAcceleration == accel
    previous = action


def test_big_filter_holds_curvature_below_lateral_control_speed():
  prev = log.ModelDataV2.Action(desiredCurvature=0.004)
  action = stock_modeld.get_action_from_model({"action": np.array([[0.03, 0.5]])}, prev, 0.3, 0.3, 0.2)
  smooth_big_model_action(action, prev, SimpleNamespace(chestnut=True), 0.1)
  assert action.desiredCurvature == pytest.approx(prev.desiredCurvature)


def test_stock_small_model_can_override_its_filter():
  prev = log.ModelDataV2.Action(desiredCurvature=0.004)
  output = {"action": np.array([[0.03, 0.5]])}
  action = stock_modeld.get_action_from_model(output, prev, 0.3, 0.3, 1.0, lat_smooth_seconds=0.1)
  assert action.desiredCurvature == pytest.approx(smooth_value(0.03, prev.desiredCurvature, 0.1))
