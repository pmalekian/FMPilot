"""FM Pilot Higher Torque: Off is AP, Headroom raises only the max request, Full scales every request, changes blend."""
from types import SimpleNamespace

import pytest

from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.fm_torque_mode import (BLEND_S, FULL, HEADROOM, OFF, PARAM_MODE, FmTorqueMode,
                                                                        high_torque_ratio)
from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP

BLEND_STEPS = int(BLEND_S / DT_CTRL) + 2


class FakeParams:
  def __init__(self, mode=OFF):
    self.values = {PARAM_MODE: mode}

  def get(self, key, return_default=False):
    return self.values.get(key)


def make(mode=OFF, brand='rivian', panda=True):
  CP = SimpleNamespace(brand=brand)
  CP_SP = SimpleNamespace(flags=RivianFlagsSP.HIGH_TORQUE_PANDA.value if panda else 0)
  params = FakeParams(mode)
  return FmTorqueMode(CP, CP_SP, params), params


def run(fm, v=4.0, steps=1):
  out = (1.0, 1.0)
  for _ in range(steps):
    out = fm.update(v)
  return out


def test_ratio_follows_the_two_curves():
  assert high_torque_ratio(0.0) == pytest.approx(481 / 385)
  assert high_torque_ratio(9.0) == pytest.approx(481 / 385)
  assert high_torque_ratio(13.0) == pytest.approx(406 / 350)
  assert high_torque_ratio(17.0) == pytest.approx(1.0)
  assert high_torque_ratio(25.0) == 1.0
  assert high_torque_ratio(35.0) == 1.0
  for v in [x * 0.25 for x in range(160)]:
    assert high_torque_ratio(v) >= 1.0


@pytest.mark.parametrize("mode", [OFF, HEADROOM, FULL])
def test_no_panda_flag_is_always_ap(mode):
  fm, _ = make(mode, panda=False)
  assert run(fm, steps=BLEND_STEPS) == (1.0, 1.0)


@pytest.mark.parametrize("mode", [OFF, HEADROOM, FULL])
def test_other_brands_are_always_ap(mode):
  fm, _ = make(mode, brand='tesla')
  assert run(fm, steps=BLEND_STEPS) == (1.0, 1.0)


def test_off_is_ap_at_every_speed():
  fm, _ = make(OFF)
  for v in (0.0, 4.0, 9.0, 13.0, 20.0, 30.0):
    assert run(fm, v=v, steps=BLEND_STEPS) == (1.0, 1.0)


def test_headroom_raises_only_the_max():
  fm, _ = make(HEADROOM)
  max_req, scale = run(fm, steps=BLEND_STEPS)
  assert max_req == pytest.approx(481 / 385)
  assert scale == 1.0
  assert run(fm, v=30.0) == (1.0, 1.0)  # highway unchanged


def test_full_scales_every_request():
  fm, _ = make(FULL)
  max_req, scale = run(fm, steps=BLEND_STEPS)
  assert max_req == 1.0
  assert scale == pytest.approx(481 / 385)


def test_mode_change_blends_over_one_second():
  fm, params = make(OFF)
  run(fm, steps=10)
  params.values[PARAM_MODE] = FULL
  fm.read_params()
  extra = 481 / 385 - 1
  prev = 1.0
  for i in range(BLEND_STEPS):
    _, scale = fm.update(4.0)
    assert scale - prev <= extra * DT_CTRL / BLEND_S + 1e-9, f"stepped at frame {i}"
    prev = scale
  assert prev == pytest.approx(1 + extra)
  # half way through a switch back, both are partly blended
  params.values[PARAM_MODE] = HEADROOM
  fm.read_params()
  max_req, scale = run(fm, steps=int(0.5 / DT_CTRL))
  assert max_req == pytest.approx(1 + extra / 2, abs=0.01)
  assert scale == pytest.approx(1 + extra / 2, abs=0.01)


def test_settings_reread_once_per_second():
  fm, params = make(OFF)
  run(fm, steps=5)
  params.values[PARAM_MODE] = HEADROOM
  run(fm, steps=int(1.0 / DT_CTRL) + BLEND_STEPS)
  assert fm.mode == HEADROOM


@pytest.mark.parametrize("bad", [None, 3, -1, "x"])
def test_bad_setting_is_off(bad):
  fm, _ = make(bad)
  assert fm.mode == OFF
  assert run(fm, steps=BLEND_STEPS) == (1.0, 1.0)
