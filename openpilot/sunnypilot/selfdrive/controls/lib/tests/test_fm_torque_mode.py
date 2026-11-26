"""FM Pilot Max Torque, Headroom: at 385 (or another car) the controller's max request stays 1.0; above it the max
request follows the higher curve at low speed, blends in over 1 s, and requests are never scaled."""
from types import SimpleNamespace

import pytest

from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.fm_torque_mode import BLEND_S, FmTorqueMode, high_torque_ratio
from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP, RivianSafetyFlagsSP, max_torque_level

BLEND_STEPS = int(BLEND_S / DT_CTRL) + 2


def make(max_torque=580, brand='rivian'):
  CP = SimpleNamespace(brand=brand)
  level = max_torque_level(max_torque)
  safety_param = (RivianSafetyFlagsSP.HIGH_TORQUE | (level << RivianSafetyFlagsSP.TORQUE_LEVEL_SHIFT)) if level >= 1 else 0
  CP_SP = SimpleNamespace(flags=RivianFlagsSP.HIGH_TORQUE_PANDA.value if level >= 1 else 0, safetyParam=safety_param)
  return FmTorqueMode(CP, CP_SP)


def run(fm, v=4.0, steps=1):
  out = 1.0
  for _ in range(steps):
    out = fm.update(v)
  return out


def test_ratio_follows_the_two_curves():
  assert high_torque_ratio(0.0, 580) == pytest.approx(580 / 385)
  assert high_torque_ratio(9.0, 580) == pytest.approx(580 / 385)
  assert high_torque_ratio(13.0, 580) == pytest.approx(round((580 + 332) / 2) / 350)
  assert high_torque_ratio(17.0, 580) == pytest.approx(1.0)
  assert high_torque_ratio(25.0, 580) == 1.0
  for cap in range(400, 581, 15):
    for v in [x * 0.25 for x in range(160)]:
      assert high_torque_ratio(v, cap) >= 1.0


@pytest.mark.parametrize("max_torque", [385, 300, 392])
def test_stock_is_always_ap(max_torque):
  fm = make(max_torque)
  assert not fm.available
  for v in (0.0, 4.0, 9.0, 13.0, 30.0):
    assert run(fm, v=v, steps=BLEND_STEPS) == 1.0


def test_other_brands_are_always_ap():
  assert run(make(580, brand='tesla'), steps=BLEND_STEPS) == 1.0


@pytest.mark.parametrize("max_torque", [400, 475, 580])
def test_headroom_raises_only_the_max(max_torque):
  fm = make(max_torque)
  assert run(fm, steps=BLEND_STEPS) == pytest.approx(max_torque / 385)
  assert run(fm, v=30.0) == 1.0  # highway unchanged


def test_blends_in_over_a_second():
  fm = make(580)
  first = fm.update(4.0)
  assert 1.0 < first < 1.0 + (580 / 385 - 1.0) * 0.02
  half = run(fm, steps=int(0.5 / DT_CTRL) - 1)
  assert half == pytest.approx(1.0 + (580 / 385 - 1.0) * 0.5, abs=1e-6)
  assert run(fm, steps=BLEND_STEPS) == pytest.approx(580 / 385)


@pytest.mark.parametrize("v_ego", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_v_ego_returns_one(v_ego):
  fm = make(580)
  run(fm, v=4.0, steps=BLEND_STEPS)
  assert fm.update(v_ego) == 1.0
