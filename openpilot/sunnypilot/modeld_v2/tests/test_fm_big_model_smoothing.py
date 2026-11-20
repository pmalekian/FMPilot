"""FM Pilot: Lateral Smoothing (Big Models), FM_BigModelSmoothing. Ported from zoompilot's BigModelSmoothing tests
(zoompilot/zoompilot 6cb59d3, MIT), plus FM's own: the stock action path a Jetlink big model uses, and the settings row."""
import inspect
import json
import os
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value
from openpilot.sunnypilot.modeld_v2.fm_big_model_smoothing import (BIG_MODEL_LAT_SMOOTH_SECONDS, PARAM_BIG_MODEL_SMOOTHING,
                                                                   big_lat_smooth_seconds, smooth_curvature)

SUNNYLINK = os.path.join(os.path.dirname(__file__), "..", "..", "sunnylink")


def _action(curvature: float) -> log.ModelDataV2.Action:
  return log.ModelDataV2.Action(desiredCurvature=curvature, desiredAcceleration=0.5)


class TestBigLatSmoothSeconds:
  def test_off_never_smooths(self):
    assert big_lat_smooth_seconds(False, SimpleNamespace(chestnut=True, LAT_SMOOTH_SECONDS=0.0)) == 0.0

  def test_on_smooths_a_big_model_that_smooths_none(self):
    assert big_lat_smooth_seconds(True, SimpleNamespace(chestnut=True, LAT_SMOOTH_SECONDS=0.0)) == BIG_MODEL_LAT_SMOOTH_SECONDS
    # stock modeld's ModelState has no LAT_SMOOTH_SECONDS attribute (it is a module constant there)
    assert big_lat_smooth_seconds(True, SimpleNamespace(chestnut=True)) == BIG_MODEL_LAT_SMOOTH_SECONDS

  def test_small_models_are_left_to_their_own_setting(self):
    assert big_lat_smooth_seconds(True, SimpleNamespace(chestnut=False, LAT_SMOOTH_SECONDS=0.0)) == 0.0
    assert big_lat_smooth_seconds(True, SimpleNamespace(LAT_SMOOTH_SECONDS=0.0)) == 0.0

  def test_a_big_model_that_already_smooths_is_not_smoothed_twice(self):
    assert big_lat_smooth_seconds(True, SimpleNamespace(chestnut=True, LAT_SMOOTH_SECONDS=0.2)) == 0.0

  def test_jetlinks_big_model_gets_stock_modelds_zero_smoothing(self):
    """A Jetlink big model reads LAT_SMOOTH_SECONDS from stock modeld (jetlink stock.py model_face), so it qualifies."""
    from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
    assert LAT_SMOOTH_SECONDS == 0.0
    assert big_lat_smooth_seconds(True, SimpleNamespace(chestnut=True, LAT_SMOOTH_SECONDS=LAT_SMOOTH_SECONDS)) > 0.0


class TestSmoothCurvature:
  def test_matches_a_bundles_smoothing(self):
    action = _action(0.01)
    smooth_curvature(action, _action(0.0), BIG_MODEL_LAT_SMOOTH_SECONDS)
    assert action.desiredCurvature == pytest.approx(smooth_value(0.01, 0.0, BIG_MODEL_LAT_SMOOTH_SECONDS))
    assert 0.0 < action.desiredCurvature < 0.01
    assert action.desiredAcceleration == 0.5  # longitudinal untouched

  def test_zero_leaves_the_action_alone(self):
    action = _action(0.01)
    smooth_curvature(action, _action(0.0), 0.0)
    assert action.desiredCurvature == pytest.approx(0.01)

  def test_keeps_a_held_curvature(self):
    """Below the lateral control speed the action holds the previous curvature; smoothing keeps it."""
    action = _action(0.004)
    smooth_curvature(action, _action(0.004), BIG_MODEL_LAT_SMOOTH_SECONDS)
    assert action.desiredCurvature == pytest.approx(0.004)

  def test_calms_a_1hz_wiggle(self):
    """The case it is for: a ~1 Hz curvature wiggle on a straight comes out smaller."""
    import math
    prev, peak_raw, peak_smoothed = _action(0.0), 0.0, 0.0
    for i in range(200):  # 10 s at 20 Hz
      raw = 0.002 * math.sin(2 * math.pi * 1.0 * i * 0.05)
      action = _action(raw)
      smooth_curvature(action, prev, BIG_MODEL_LAT_SMOOTH_SECONDS)
      prev = action
      if i >= 40:
        peak_raw, peak_smoothed = max(peak_raw, abs(raw)), max(peak_smoothed, abs(action.desiredCurvature))
    assert peak_smoothed < 0.9 * peak_raw


class TestWiring:
  @pytest.mark.parametrize("module", ["openpilot.sunnypilot.modeld_v2.modeld", "openpilot.selfdrive.modeld.modeld"])
  def test_both_modelds_smooth_after_the_action_and_before_prev_action(self, module):
    import importlib
    src = inspect.getsource(importlib.import_module(module))
    i_action = src.index("get_action_from_model(model_output, prev_action")
    i_smooth = src.index("smooth_curvature(action, prev_action, big_lat_smooth_seconds(big_model_smoothing, model))")
    i_prev = src.index("prev_action = action", i_action)
    assert i_action < i_smooth < i_prev
    assert "params.get_bool(PARAM_BIG_MODEL_SMOOTHING)" in src

  def test_param_is_off_by_default(self):
    params = Params()
    params.remove(PARAM_BIG_MODEL_SMOOTHING)
    assert params.get_bool(PARAM_BIG_MODEL_SMOOTHING) is False

  def test_settings_row_sits_right_under_the_small_model_row(self):
    with open(os.path.join(SUNNYLINK, "settings_ui.json")) as f:
      ui = json.load(f)

    def walk(node):
      if isinstance(node, dict):
        if isinstance(node.get("items"), list):
          yield node["items"]
        for v in node.values():
          yield from walk(v)
      elif isinstance(node, list):
        for v in node:
          yield from walk(v)

    for items in walk(ui):
      keys = [it.get("key") for it in items if isinstance(it, dict)]
      if "FM_LateralSmoothingEnabled" in keys:
        i = keys.index("FM_LateralSmoothingEnabled")
        assert keys[i + 1] == PARAM_BIG_MODEL_SMOOTHING
        assert items[i]["title"] == "Lateral Smoothing (Small Models)"
        assert items[i + 1]["title"] == "Lateral Smoothing (Big Models)"
        assert items[i + 1]["widget"] == "toggle"
        return
    raise AssertionError("FM_LateralSmoothingEnabled not found in settings_ui.json")
