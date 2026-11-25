"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import pytest

from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.modeld import modeld as stock_modeld
from openpilot.sunnypilot.modeld_v2.fm_pull_away import PARAM_CRAWL_SPEED, PARAM_STRAIGHT_PULL_AWAY, StraightPullAway
from openpilot.sunnypilot.modeld_v2.modeld import ModelState


@pytest.fixture(params=["stock", "v2"])
def action(request):
  if request.param == "stock":
    return stock_modeld.get_action_from_model
  model = ModelState.__new__(ModelState)
  model.generation = None
  model.LONG_SMOOTH_SECONDS = stock_modeld.LONG_SMOOTH_SECONDS
  return model.get_action_from_model


@pytest.mark.parametrize("v_ego, floor", [(0.8, 1.0), (0.8, 2.5), (2.0, 2.5), (3.0, 2.5)])
def test_action_head_uses_speed_floor(action, v_ego, floor):
  output = {"action": np.array([[0.02, 0.5]])}
  previous = log.ModelDataV2.Action()
  stock = action(output, previous, 0.3, 0.3, v_ego)
  assert stock.desiredCurvature == pytest.approx(0.02 / max(1.0, v_ego)**2)
  adjusted = action(output, previous, 0.3, 0.3, v_ego, action_speed_floor=floor)
  assert adjusted.desiredCurvature == pytest.approx(0.02 / max(floor, v_ego)**2)
  assert adjusted.desiredAcceleration == stock.desiredAcceleration
  assert adjusted.shouldStop == stock.shouldStop


class TestFloorSettings(OpenpilotTestCase):
  def test_fresh_params_and_retained_toggles(self):
    params = Params()
    state = car.CarState.new_message()
    floor = StraightPullAway()
    assert not params.get_bool(PARAM_CRAWL_SPEED)
    assert not params.get_bool(PARAM_STRAIGHT_PULL_AWAY)
    floor.read_params(params)
    assert floor.speed_floor(0.0, state, False) == 1.0
    assert floor.speed_floor(0.8, state, False) == 1.0

    params.put_bool(PARAM_STRAIGHT_PULL_AWAY, True, block=True)
    floor.read_params(Params())
    assert floor.speed_floor(0.8, state, False) == 1.0
    assert floor.speed_floor(0.0, state, False) == pytest.approx(2.5)
    assert floor.speed_floor(0.8, state, False) == pytest.approx(2.5)

    params.put_bool(PARAM_CRAWL_SPEED, True, block=True)
    floor.read_params(Params())
    state.rightBlinker = True
    assert floor.speed_floor(0.0, state, True) == 2.5
    assert floor.speed_floor(0.8, state, True) == 2.5
