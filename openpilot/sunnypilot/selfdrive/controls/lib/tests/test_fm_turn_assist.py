import math
from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import (FmTurnAssist, TurnDirection, PARAM_BOOST, PARAM_BOOST_LEFT,
                                                                         PARAM_BOOST_RIGHT, PARAM_HOLD, HOLD_MAX_HEADING, FADE_TIME)

LEFT, RIGHT, NONE = TurnDirection.turnLeft, TurnDirection.turnRight, TurnDirection.none
FADE_STEPS = int(FADE_TIME / DT_CTRL) + 2


class FakeParams:
  def __init__(self, **values):
    self.values = {PARAM_BOOST: False, PARAM_BOOST_LEFT: 1.1, PARAM_BOOST_RIGHT: 1.2, PARAM_HOLD: False} | values

  def get_bool(self, key):
    return bool(self.values.get(key))

  def get(self, key, return_default=False):
    return self.values.get(key)


def run(assist, k, direction, mph=10, steps=1, actual=None, pressed=False, lat=True):
  out = k
  for _ in range(steps):
    out = assist.update(k, direction, mph * CV.MPH_TO_MS, k if actual is None else actual, pressed, lat)
  return out


class TestSignConvention:
  def test_right_turn_is_positive_curvature(self):
    # desiredCurvature is positive to the right (controlsd: curvature = -VM.calc_curvature(left-positive angle))
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import turn_sign
    assert turn_sign(RIGHT) == 1
    assert turn_sign(LEFT) == -1
    assert turn_sign(NONE) == 0

  def test_left_turn_boosts_leftward_request_only(self):
    a = FmTurnAssist(FakeParams(**{PARAM_BOOST: True}))
    assert run(a, -0.08, LEFT, steps=FADE_STEPS) == pytest.approx(-0.08 * 1.1)
    b = FmTurnAssist(FakeParams(**{PARAM_BOOST: True}))
    assert run(b, 0.02, LEFT, steps=FADE_STEPS) == 0.02  # rightward request in a left turn is left alone

  def test_hold_never_pulls_away_from_the_turn(self):
    a = FmTurnAssist(FakeParams(**{PARAM_HOLD: True}))
    run(a, -0.09, LEFT, steps=FADE_STEPS, actual=-0.001)
    assert run(a, -0.03, LEFT, actual=-0.001) < -0.03  # holds the left turn tighter, never to the right


class TestTurnBoost:
  def test_off_by_default(self):
    a = FmTurnAssist(FakeParams())
    assert run(a, -0.08, RIGHT, steps=FADE_STEPS) == -0.08

  def test_per_side_gain_after_fade_in(self):
    a = FmTurnAssist(FakeParams(**{PARAM_BOOST: True}))
    assert run(a, 0.08, RIGHT, steps=FADE_STEPS) == pytest.approx(0.08 * 1.2)
    b = FmTurnAssist(FakeParams(**{PARAM_BOOST: True}))
    assert run(b, -0.08, LEFT, steps=FADE_STEPS) == pytest.approx(-0.08 * 1.1)

  def test_fades_in(self):
    a = FmTurnAssist(FakeParams(**{PARAM_BOOST: True}))
    first = run(a, 0.08, RIGHT)
    assert 0.08 < first < 0.08 * 1.2  # a little boost on the first frame, not all of it

  def test_no_turn_no_boost(self):
    a = FmTurnAssist(FakeParams(**{PARAM_BOOST: True}))
    assert run(a, -0.08, NONE, steps=FADE_STEPS) == -0.08
    assert run(a, -0.08, None, steps=FADE_STEPS) == -0.08

  def test_never_boosts_away_from_the_turn(self):
    a = FmTurnAssist(FakeParams(**{PARAM_BOOST: True}))
    assert run(a, -0.02, RIGHT, steps=FADE_STEPS) == -0.02  # right turn, request steering left (curvature is positive to the right)

  def test_fades_out_with_speed(self):
    a = FmTurnAssist(FakeParams(**{PARAM_BOOST: True}))
    assert run(a, 0.02, RIGHT, mph=25, steps=FADE_STEPS) == 0.02
    b = FmTurnAssist(FakeParams(**{PARAM_BOOST: True}))
    half = run(b, 0.02, RIGHT, mph=17.5, steps=FADE_STEPS)
    assert half == pytest.approx(0.02 * 1.1)

  def test_gain_clipped(self):
    a = FmTurnAssist(FakeParams(**{PARAM_BOOST: True, PARAM_BOOST_RIGHT: 3.0}))
    assert run(a, 0.08, RIGHT, steps=FADE_STEPS) == pytest.approx(0.08 * 1.5)

  def test_not_steering_no_change(self):
    a = FmTurnAssist(FakeParams(**{PARAM_BOOST: True}))
    assert run(a, 0.08, RIGHT, steps=FADE_STEPS, lat=False) == 0.08


class TestTurnHold:
  def hold(self):
    return FmTurnAssist(FakeParams(**{PARAM_HOLD: True}))

  def test_holds_when_model_lets_go(self):
    a = self.hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    out = run(a, 0.03, RIGHT, actual=0.001)  # model eases to a 33 m radius mid-turn
    assert out == pytest.approx(0.09 * 0.9, rel=0.01)

  def test_floor_decays(self):
    a = self.hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    out = run(a, 0.03, RIGHT, steps=int(3 / DT_CTRL), actual=0.001)
    assert abs(out) < 0.09 * 0.9 - 0.09 * 0.15 * 2.5

  def test_follows_model_when_it_asks_more(self):
    a = self.hold()
    run(a, 0.05, RIGHT, steps=FADE_STEPS, actual=0.001)
    assert run(a, 0.10, RIGHT, actual=0.001) == 0.10

  def test_releases_after_heading(self):
    a = self.hold()
    k, v = 0.1, 10 * CV.MPH_TO_MS
    frames = int(HOLD_MAX_HEADING / (abs(k) * v * DT_CTRL)) + 5
    run(a, k, RIGHT, steps=frames)
    assert a.hold_released
    assert run(a, 0.02, RIGHT, actual=0.02) == 0.02

  def test_releases_when_driver_steers(self):
    a = self.hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    assert run(a, 0.03, RIGHT, pressed=True) == 0.03
    assert run(a, 0.03, RIGHT) == 0.03  # stays released for this turn

  def test_new_turn_resets(self):
    a = self.hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.03, RIGHT, pressed=True)
    run(a, 0.0, NONE)
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    assert run(a, 0.03, RIGHT, actual=0.001) == pytest.approx(0.081, rel=0.01)

  def test_heading_math(self):
    assert math.degrees(HOLD_MAX_HEADING) == pytest.approx(75)


class TestFmLateralWiring:
  def test_turn_direction_from_modeld(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_lateral import FmLateral

    class FakeSM(dict):
      def __init__(self, direction):
        super().__init__(modelDataV2SP=SimpleNamespace(laneTurnDirection=direction))
        self.seen = {'modelDataV2SP': True}

      def update(self, timeout):
        pass

    params = FakeParams(**{PARAM_BOOST: True})
    params.values.update({"LaneGuardEnabled": False, "RivianOfftrackCompensationEnabled": False, "IsRhdDetected": False})
    CP = SimpleNamespace(brand='rivian', wheelbase=3.08)
    lat = FmLateral(CP, params, turn_sm=FakeSM(RIGHT))
    CS = SimpleNamespace(vEgo=10 * CV.MPH_TO_MS, steeringPressed=False, leftBlinker=False, rightBlinker=True)
    model = SimpleNamespace()
    lat.lane_guard.update = lambda *a, **k: 0.0
    out = 0.08
    for _ in range(FADE_STEPS):
      out = lat.update(0.08, model, CS, True, True, 0.08)
    assert out == pytest.approx(0.08 * 1.2)
