"""FM Pilot: Blendful's hook in modeld_v2's get_action_from_model. Not selected must leave the action untouched."""
from functools import partial
from typing import Any

import numpy as np

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.modeld_v2.blendful import Blendful
from openpilot.sunnypilot.modeld_v2.modeld import ModelState
from openpilot.sunnypilot.modeld_v2.tests.test_blendful import frame


class MockStruct:
  def __init__(self, **kwargs):
    for k, v in kwargs.items():
      setattr(self, k, v)


def _state(generation=12):
  return MockStruct(PLANPLUS_CONTROL=1.0, LONG_SMOOTH_SECONDS=0.3, LAT_SMOOTH_SECONDS=0.1, MIN_LAT_CONTROL_SPEED=0.3,
                    mlsim=True, generation=generation, constants=MockStruct(T_IDXS=np.arange(33), DESIRE_LEN=8))


def _output(center=0.5, lat_accel=0.3):
  out: dict = frame(center=center)
  out['action'] = np.array([[lat_accel, 0.2]], dtype=np.float32)  # [lateral accel, accel], as action-head models output
  return out


def _run(blendful_fn, v_ego=20.0, steps=200, center=0.5, generation=12):
  state: Any = _state(generation)
  prev = log.ModelDataV2.Action()
  actions = []
  for _ in range(steps):
    out = _output(center)
    fn = None if blendful_fn is None else partial(blendful_fn, out, v_ego, True)
    prev = ModelState.get_action_from_model(state, out, prev, 0.0, 0.0, v_ego, fn)
    actions.append((prev.desiredCurvature, prev.desiredAcceleration, prev.shouldStop))
  return actions


def test_not_selected_is_identical_to_no_blendful():
  bf = Blendful()
  for v_ego in (0.0, 3.0, 8.0, 20.0, 31.0):
    for generation in (None, 12):
      without = _run(None, v_ego=v_ego, generation=generation)
      with_off = _run(partial(bf.update, enabled=False), v_ego=v_ego, generation=generation)
      assert without == with_off  # exact, not approximately


def test_selected_pulls_toward_lane_center():
  bf = Blendful()
  without = _run(None, steps=int(5 / DT_MDL))
  with_on = _run(partial(bf.update, enabled=True), steps=int(5 / DT_MDL))
  assert bf.trust > 0.9
  assert with_on[-1][0] > without[-1][0]  # lane center is to the right (+), so more right curvature
  assert with_on[-1][1] == without[-1][1]  # acceleration untouched
