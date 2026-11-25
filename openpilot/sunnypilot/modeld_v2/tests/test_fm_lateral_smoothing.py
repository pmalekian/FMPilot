"""FM Pilot: Model Lateral Smoothing tests for small and big models (FM_SmallModelLatSmoothing, FM_BigModelLatSmoothing).

Includes tests ported from zoompilot's BigModelSmoothing tests (Zeph Leggett, zoompilot/zoompilot 6cb59d3, MIT) plus
FM Pilot's own tests for the reshaped Model Default / 0.00 s / 0.10 s drop-downs, both modelds, and Sunnylink UI.
"""
import importlib
import inspect
import json
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value
from openpilot.selfdrive.modeld import modeld as stock_modeld
from openpilot.sunnypilot.modeld_v2.fm_lateral_smoothing import (
  BIG_MODEL_LAT_SMOOTH_SECONDS,
  MODE_0_00,
  MODE_0_10,
  MODE_DEFAULT,
  PARAM_BIG_MODEL_SMOOTHING,
  PARAM_SMALL_MODEL_SMOOTHING,
  big_bundle_lat_smooth_seconds,
  big_lat_smooth_seconds,
  read_smoothing_mode,
  small_lat_smooth_seconds,
  smooth_curvature,
)
from openpilot.sunnypilot.modeld_v2.modeld import ModelState

SUNNYLINK = os.path.join(os.path.dirname(__file__), "..", "..", "sunnylink")


class FakeParams:
  def __init__(self, values=None):
    self.values = values or {}

  def get(self, key, return_default=False):
    assert return_default
    return self.values.get(key)


def _action(curvature: float) -> log.ModelDataV2.Action:
  return log.ModelDataV2.Action(desiredCurvature=curvature, desiredAcceleration=0.5)


class TestReadSmoothingMode:
  @pytest.mark.parametrize("raw,expected", [
    (0, MODE_DEFAULT),
    (1, MODE_0_00),
    (2, MODE_0_10),
    ("2", MODE_0_10),
    (None, MODE_DEFAULT),
    (-1, MODE_DEFAULT),
    (3, MODE_DEFAULT),
    ("bad", MODE_DEFAULT),
  ])
  def test_parses_valid_modes_and_falls_back_to_default(self, raw, expected):
    assert read_smoothing_mode(FakeParams({PARAM_SMALL_MODEL_SMOOTHING: raw}), PARAM_SMALL_MODEL_SMOOTHING) == expected


class TestSmallLatSmoothSeconds:
  @pytest.mark.parametrize("bundle", [0.0, 0.1, 0.3])
  def test_model_default_keeps_bundle_value(self, bundle):
    assert small_lat_smooth_seconds(MODE_DEFAULT, bundle) == bundle

  @pytest.mark.parametrize("bundle", [0.0, 0.1, 0.3])
  def test_0_00_and_0_10_replace_bundle_value(self, bundle):
    assert small_lat_smooth_seconds(MODE_0_00, bundle) == 0.0
    assert small_lat_smooth_seconds(MODE_0_10, bundle) == 0.1

  def test_unknown_mode_falls_back_to_bundle(self):
    assert small_lat_smooth_seconds(99, 0.15) == 0.15


class TestBigLatSmoothSeconds:
  @pytest.mark.parametrize("mode", [MODE_DEFAULT, MODE_0_00])
  def test_default_and_0_00_never_add_post_action_smoothing(self, mode):
    assert big_lat_smooth_seconds(mode, SimpleNamespace(chestnut=True, LAT_SMOOTH_SECONDS=0.0)) == 0.0

  def test_0_10_smooths_a_big_model_that_smooths_none(self):
    assert big_lat_smooth_seconds(MODE_0_10, SimpleNamespace(chestnut=True, LAT_SMOOTH_SECONDS=0.0)) == BIG_MODEL_LAT_SMOOTH_SECONDS
    # stock modeld's ModelState has no LAT_SMOOTH_SECONDS attribute (it is a module constant there)
    assert big_lat_smooth_seconds(MODE_0_10, SimpleNamespace(chestnut=True)) == BIG_MODEL_LAT_SMOOTH_SECONDS

  def test_small_models_are_left_to_their_own_setting(self):
    assert big_lat_smooth_seconds(MODE_0_10, SimpleNamespace(chestnut=False, LAT_SMOOTH_SECONDS=0.0)) == 0.0
    assert big_lat_smooth_seconds(MODE_0_10, SimpleNamespace(LAT_SMOOTH_SECONDS=0.0)) == 0.0

  def test_a_big_model_that_already_smooths_is_not_smoothed_twice(self):
    assert big_lat_smooth_seconds(MODE_0_10, SimpleNamespace(chestnut=True, LAT_SMOOTH_SECONDS=0.2)) == 0.0

  def test_big_bundle_helper_clears_only_on_0_00(self):
    assert big_bundle_lat_smooth_seconds(MODE_DEFAULT, 0.1) == 0.1
    assert big_bundle_lat_smooth_seconds(MODE_0_00, 0.1) == 0.0
    assert big_bundle_lat_smooth_seconds(MODE_0_10, 0.0) == 0.0
    assert big_bundle_lat_smooth_seconds(MODE_0_10, 0.1) == 0.1

  def test_jetlinks_big_model_gets_stock_modelds_zero_smoothing(self):
    """A Jetlink big model reads LAT_SMOOTH_SECONDS from stock modeld (jetlink stock.py model_face), so it qualifies."""
    assert stock_modeld.LAT_SMOOTH_SECONDS == 0.0
    assert big_lat_smooth_seconds(MODE_0_10, SimpleNamespace(chestnut=True, LAT_SMOOTH_SECONDS=stock_modeld.LAT_SMOOTH_SECONDS)) > 0.0


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
    prev, peak_raw, peak_smoothed = _action(0.0), 0.0, 0.0
    for i in range(200):  # 10 s at 20 Hz
      raw = 0.002 * math.sin(2 * math.pi * 1.0 * i * 0.05)
      action = _action(raw)
      smooth_curvature(action, prev, BIG_MODEL_LAT_SMOOTH_SECONDS)
      prev = action
      if i >= 40:
        peak_raw, peak_smoothed = max(peak_raw, abs(raw)), max(peak_smoothed, abs(action.desiredCurvature))
    assert peak_smoothed < 0.9 * peak_raw


class TestActionSmoothing:
  def _step_modeld_v2(self, smooth_seconds: float) -> float:
    class S:
      pass
    state = S()
    state.__dict__.update(PLANPLUS_CONTROL=1.0, LONG_SMOOTH_SECONDS=0.0, LAT_SMOOTH_SECONDS=smooth_seconds,
                          MIN_LAT_CONTROL_SPEED=0.3, mlsim=True, generation=12)
    out = {'action': np.array([[1.0, 0.0]], dtype=np.float32)}
    return ModelState.get_action_from_model(state, out, log.ModelDataV2.Action(), 0.0, 0.0, 10.0).desiredCurvature

  def test_modeld_v2_zero_and_0_10(self):
    zero = self._step_modeld_v2(0.0)
    one = self._step_modeld_v2(0.1)
    assert zero == pytest.approx(1.0 / 10.0 ** 2)
    assert one == pytest.approx((1 - np.exp(-DT_MDL / 0.1)) * zero, rel=1e-4)

  def test_stock_modeld_action_accepts_lat_smooth_seconds_keyword_only(self):
    out = {'action': np.array([[1.0, 0.0]], dtype=np.float32)}
    zero = stock_modeld.get_action_from_model(out, log.ModelDataV2.Action(), 0.0, 0.0, 10.0, lat_smooth_seconds=0.0).desiredCurvature
    one = stock_modeld.get_action_from_model(out, log.ModelDataV2.Action(), 0.0, 0.0, 10.0, lat_smooth_seconds=0.1).desiredCurvature
    assert zero == pytest.approx(1.0 / 10.0 ** 2)
    assert one == pytest.approx((1 - np.exp(-DT_MDL / 0.1)) * zero, rel=1e-4)
    # keyword-only protects against Jetlink/modeld_v2 passing a 6th positional argument
    with pytest.raises(TypeError):
      stock_modeld.get_action_from_model(out, log.ModelDataV2.Action(), 0.0, 0.0, 10.0, 0.1)  # type: ignore[misc]


class TestWiring:
  @pytest.mark.parametrize("module", ["openpilot.sunnypilot.modeld_v2.modeld", "openpilot.selfdrive.modeld.modeld"])
  def test_both_modelds_smooth_after_the_action_and_before_prev_action(self, module):
    src = inspect.getsource(importlib.import_module(module))
    i_action = src.index("get_action_from_model(model_output, prev_action")
    i_smooth = src.index("smooth_curvature(action, prev_action, big_lat_smooth_seconds(big_model_smoothing, model))")
    i_prev = src.index("prev_action = action", i_action)
    assert i_action < i_smooth < i_prev
    assert "read_smoothing_mode(params, PARAM_BIG_MODEL_SMOOTHING)" in src

  def test_params_default_to_model_default(self):
    params = Params()
    for key in (PARAM_SMALL_MODEL_SMOOTHING, PARAM_BIG_MODEL_SMOOTHING):
      params.remove(key)
      assert read_smoothing_mode(params, key) == MODE_DEFAULT

  def test_settings_ui_has_both_dropdowns_with_difference_notes(self):
    with open(os.path.join(SUNNYLINK, "settings_ui.json")) as f:
      ui = json.load(f)

    for panel in ui["panels"]:
      if panel.get("id") != "steering":
        continue
      for section in panel.get("sections", []):
        if section.get("id") != "fm_model_smoothing":
          continue
        items = section.get("items", [])
        assert [it["key"] for it in items] == [PARAM_SMALL_MODEL_SMOOTHING, PARAM_BIG_MODEL_SMOOTHING]
        expected_opts = [{"value": 0, "label": "Model Default"}, {"value": 1, "label": "0.00 s"}, {"value": 2, "label": "0.10 s"}]
        for it in items:
          assert it["widget"] == "multiple_button"
          assert it["options"] == expected_opts
        assert "lateral delay" in items[0]["description"].lower()
        assert "not added" in items[1]["description"].lower()
        return
    raise AssertionError("fm_model_smoothing section not found in settings_ui.json")
