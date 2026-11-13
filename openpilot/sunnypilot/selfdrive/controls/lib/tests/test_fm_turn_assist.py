import math
from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import (FmTurnAssist, TurnDirection, PARAM_BOOST, PARAM_BOOST_LEFT,
                                                                         PARAM_BOOST_RIGHT, PARAM_HOLD, HOLD_MAX_HEADING, FADE_TIME,
                                                                         PARAM_LEAD, PARAM_MIN_RADIUS, MIN_RADIUS_DEFAULT, PARAM_HOLD_PATH,
                                                                         PARAM_HOLD_DRIVER, PARAM_LEAD_MIN_SPEED, PARAM_HOLD_EARLY,
                                                                         PARAM_PHASE_IN, PHASE_IN_TAU, PARAM_LEAD_ADAPTIVE, PARAM_LEAD_GATE,
                                                                         PARAM_LEAD_GATE_DIST, plan_circle_curvature, PARAM_HOLD_DEBOUNCE,
                                                                         HOLD_AGAINST_TIME)

LEFT, RIGHT, NONE = TurnDirection.turnLeft, TurnDirection.turnRight, TurnDirection.none
FADE_STEPS = int(FADE_TIME / DT_CTRL) + 2


class FakeParams:
  def __init__(self, **values):
    self.values = {PARAM_BOOST: False, PARAM_BOOST_LEFT: 1.1, PARAM_BOOST_RIGHT: 1.2, PARAM_HOLD: False, PARAM_LEAD: False,
                   PARAM_HOLD_PATH: False, PARAM_HOLD_DRIVER: False, PARAM_HOLD_EARLY: False, PARAM_LEAD_MIN_SPEED: 7.0,
                   PARAM_MIN_RADIUS: MIN_RADIUS_DEFAULT, PARAM_PHASE_IN: False,
                   PARAM_LEAD_ADAPTIVE: False, PARAM_LEAD_GATE: False, PARAM_LEAD_GATE_DIST: 5.0,
                   PARAM_HOLD_DEBOUNCE: False} | values

  def get_bool(self, key):
    return bool(self.values.get(key))

  def get(self, key, return_default=False):
    return self.values.get(key)


def run(assist, k, direction, mph=10, steps=1, actual=None, pressed=False, lat=True, model=None, a_ego=0.0, lane_changing=False,
        torque=0.0):
  out = k
  for _ in range(steps):
    out = assist.update(k, direction, mph * CV.MPH_TO_MS, k if actual is None else actual, pressed, lat, model_v2=model,
                        a_ego=a_ego, lane_changing=lane_changing, steering_torque=torque)
  return out


def path(radius=None, start=2.0, side=1, length=60):
  """Planned path: straight for `start` m, then a 90 degree corner of `radius` m turning right (side=1) or left (side=-1),
  then straight out of the corner."""
  xs, ys = [], []
  for i in range(length + 1):
    s = float(i)
    if radius is None or s <= start:
      x, y = s, 0.0
    elif s <= start + radius * math.pi / 2:
      th = (s - start) / radius
      x, y = start + radius * math.sin(th), side * radius * (1.0 - math.cos(th))
    else:
      x, y = start + radius, side * (radius + s - start - radius * math.pi / 2)
    xs.append(x)
    ys.append(y)
  return SimpleNamespace(position=SimpleNamespace(x=xs, y=ys))


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

  @pytest.mark.parametrize("direction, sign", [(RIGHT, 1), (LEFT, -1)])
  def test_signal_cancellation_fades_active_hold(self, direction, sign):
    a = self.hold()
    run(a, sign * 0.09, direction, steps=FADE_STEPS, actual=0.001)
    held = run(a, sign * 0.03, direction, actual=0.001)
    first = run(a, sign * 0.03, NONE)
    middle = run(a, sign * 0.03, NONE, steps=FADE_STEPS // 2)
    assert sign * held > sign * first > sign * middle > 0.03
    assert run(a, sign * 0.03, NONE, steps=FADE_STEPS) == sign * 0.03

  @pytest.mark.parametrize("exit_options", [{"pressed": True}, {"lat": False}, {"lane_changing": True}, {"mph": 21}])
  def test_cancellation_fade_yields_to_exit_conditions(self, exit_options):
    a = self.hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.03, RIGHT)
    assert run(a, 0.03, NONE) > 0.03
    assert run(a, 0.03, NONE, **exit_options) == 0.03
    assert run(a, 0.03, NONE) == 0.03

  def test_cancelled_hold_does_not_carry_into_new_turn(self):
    a = self.hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.03, RIGHT)
    assert run(a, 0.03, NONE) > 0.03
    assert run(a, 0.03, RIGHT) == 0.03

  def test_cancellation_does_not_revive_released_hold(self):
    a = self.hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.03, RIGHT, pressed=True)
    assert run(a, 0.03, NONE) == 0.03

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

  def test_waiting_stopped_does_not_use_up_the_hold(self):
    a = self.hold()
    run(a, 0.0, RIGHT, mph=0, steps=int(20 / DT_CTRL), actual=0.0)  # 20 s waiting at the corner
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    assert run(a, 0.03, RIGHT, actual=0.001) == pytest.approx(0.081, rel=0.01)

  def test_heading_math(self):
    assert math.degrees(HOLD_MAX_HEADING) == pytest.approx(75)


class TestHoldPathSource:
  def hold(self, path_source=True, **values):
    return FmTurnAssist(FakeParams(**({PARAM_HOLD: True, PARAM_HOLD_PATH: path_source} | values)))

  def test_off_by_default(self):
    a = self.hold(path_source=False)
    assert run(a, 0.0, RIGHT, mph=1, steps=FADE_STEPS, model=path(10.0, start=1.0), actual=0.0) == 0.0

  def test_turns_from_a_crawl_when_model_says_straight(self):
    a = self.hold()
    out = run(a, 0.0, RIGHT, mph=1, steps=FADE_STEPS, model=path(10.0, start=0.0), actual=0.0)
    assert 0.02 < out <= 1 / MIN_RADIUS_DEFAULT + 1e-9

  def test_left(self):
    a = self.hold()
    assert run(a, 0.0, LEFT, mph=1, steps=FADE_STEPS, model=path(10.0, start=0.0, side=-1), actual=0.0) < -0.02

  def test_radius_cap(self):
    gentle = run(self.hold(**{PARAM_MIN_RADIUS: 20.0}), 0.0, RIGHT, mph=1, steps=FADE_STEPS, model=path(10.0, start=0.0),
                 actual=0.0)
    default = run(self.hold(), 0.0, RIGHT, mph=1, steps=FADE_STEPS, model=path(10.0, start=0.0), actual=0.0)
    assert 0.0 < gentle <= 1 / 20.0
    assert default > gentle

  def test_holds_path_turn_after_speeding_up(self):
    a = self.hold()
    asked = run(a, 0.0, RIGHT, mph=1, steps=FADE_STEPS, model=path(10.0, start=0.0), actual=0.0)
    assert run(a, 0.0, RIGHT, mph=5, model=path(10.0, start=0.0), actual=0.0) == pytest.approx(asked * 0.9, rel=0.02)

  def test_only_below_2_mph(self):
    a = self.hold()
    assert run(a, 0.0, RIGHT, mph=3, steps=FADE_STEPS, model=path(10.0, start=0.0), actual=0.0) == 0.0

  def test_turn_must_start_close(self):
    a = self.hold()
    assert run(a, 0.0, RIGHT, mph=1, steps=FADE_STEPS, model=path(10.0, start=8.0), actual=0.0) == 0.0

  def test_short_path_ignored(self):
    a = self.hold()
    assert run(a, 0.0, RIGHT, mph=1, steps=FADE_STEPS, model=path(10.0, start=0.0, length=6), actual=0.0) == 0.0

  def test_never_toward_the_other_side(self):
    a = self.hold()
    assert run(a, 0.0, RIGHT, mph=1, steps=FADE_STEPS, model=path(10.0, start=0.0, side=-1), actual=0.0) == 0.0

  def test_needs_turn_hold_on(self):
    a = FmTurnAssist(FakeParams(**{PARAM_HOLD_PATH: True}))
    assert run(a, 0.0, RIGHT, mph=1, steps=FADE_STEPS, model=path(10.0, start=0.0), actual=0.0) == 0.0


class TestHoldDriverAdds:
  # hand torque is positive to the LEFT; curvature positive to the RIGHT
  INTO_RIGHT, INTO_LEFT = -1.5, 1.5

  def hold(self, adds=True, **values):
    return FmTurnAssist(FakeParams(**({PARAM_HOLD: True, PARAM_HOLD_DRIVER: adds} | values)))

  def test_off_driver_still_releases(self):
    a = self.hold(adds=False)
    run(a, 0.03, RIGHT, steps=FADE_STEPS, actual=0.03)
    assert run(a, 0.03, RIGHT, pressed=True, torque=self.INTO_RIGHT, actual=0.06) == 0.03
    assert a.hold_released

  def test_steering_into_the_turn_adds(self):
    a = self.hold()
    run(a, 0.03, RIGHT, steps=FADE_STEPS, actual=0.03)
    assert run(a, 0.03, RIGHT, pressed=True, torque=self.INTO_RIGHT, actual=0.06) == pytest.approx(0.06)
    assert not a.hold_released
    assert run(a, 0.03, RIGHT, actual=0.06) == pytest.approx(0.06 * 0.9, rel=0.01)  # let go: keeps the help

  def test_left_turn(self):
    a = self.hold()
    run(a, -0.03, LEFT, steps=FADE_STEPS, actual=-0.03)
    assert run(a, -0.03, LEFT, pressed=True, torque=self.INTO_LEFT, actual=-0.06) == pytest.approx(-0.06)
    assert not a.hold_released

  def test_steering_against_the_turn_releases(self):
    a = self.hold()
    run(a, 0.06, RIGHT, steps=FADE_STEPS, actual=0.06)
    assert run(a, 0.03, RIGHT, pressed=True, torque=self.INTO_LEFT, actual=0.06) == 0.03
    assert a.hold_released

  def test_steering_in_before_the_truck_turns_keeps_the_hold(self):
    # 60:50 of route 0000002d: the driver turned the wheel in while the truck still pointed straight; the hold must stay
    a = self.hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.0)
    out = run(a, 0.03, RIGHT, pressed=True, torque=self.INTO_RIGHT, actual=0.0)
    assert not a.hold_released
    assert out == pytest.approx(0.09 * 0.9, rel=0.01)  # still holding the turn
    assert run(a, 0.03, RIGHT, pressed=True, torque=self.INTO_RIGHT, actual=0.12) == pytest.approx(0.12)
    assert not a.hold_released  # once the truck turns, its curve is adopted

  def test_off_steering_in_still_releases(self):
    a = self.hold(adds=False)
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.0)
    run(a, 0.03, RIGHT, pressed=True, torque=self.INTO_RIGHT, actual=0.0)
    assert a.hold_released

  def test_follows_your_curve_past_the_slider(self):
    # route 0000002d loops: the driver turned 5 to 6.6 m while the 8 m cap made openpilot push out; now it follows
    a = self.hold(**{PARAM_MIN_RADIUS: 8.0})
    run(a, 0.03, RIGHT, steps=FADE_STEPS, actual=0.03)
    assert run(a, 0.03, RIGHT, pressed=True, torque=self.INTO_RIGHT, actual=0.19) == pytest.approx(0.19)
    assert run(a, 0.03, RIGHT, actual=0.19) == pytest.approx(0.19 * 0.9, rel=0.02)  # let go: keeps your turn

  def test_bad_reading_guard(self):
    a = self.hold()
    run(a, 0.03, RIGHT, steps=FADE_STEPS, actual=0.03)
    assert run(a, 0.03, RIGHT, pressed=True, torque=self.INTO_RIGHT, actual=0.9) == pytest.approx(0.25)

  def test_needs_turn_hold_on(self):
    a = FmTurnAssist(FakeParams(**{PARAM_HOLD_DRIVER: True}))
    assert run(a, 0.03, RIGHT, steps=FADE_STEPS, pressed=True, torque=self.INTO_RIGHT, actual=0.06) == 0.03


class TestHoldEarlyCorrection:
  # hand torque is positive to the LEFT; curvature positive to the RIGHT
  AGAINST_RIGHT, AGAINST_LEFT, INTO_RIGHT = 1.5, -1.5, -1.5

  def hold(self, early=True, **values):
    return FmTurnAssist(FakeParams(**({PARAM_HOLD: True, PARAM_HOLD_EARLY: early} | values)))

  def test_off_still_releases(self):
    a = self.hold(early=False)
    run(a, 0.09, RIGHT, mph=3, steps=FADE_STEPS, actual=0.0)
    run(a, 0.0, RIGHT, mph=3, pressed=True, torque=self.AGAINST_RIGHT, actual=-0.005)
    assert a.hold_released

  def test_straightening_at_the_start_keeps_the_hold(self):
    a = self.hold()
    run(a, 0.09, RIGHT, mph=3, steps=FADE_STEPS, actual=0.0)
    # while you steer, the model's request passes through
    assert run(a, 0.0, RIGHT, mph=3, steps=20, pressed=True, torque=self.AGAINST_RIGHT, actual=-0.005) == 0.0
    assert not a.hold_released
    # let go: the hold carries on from the same peak, its fade paused while you steered
    assert run(a, 0.03, RIGHT, mph=3, actual=0.0) == pytest.approx(0.09 * 0.9, rel=0.01)

  def test_left_turn(self):
    a = self.hold()
    run(a, -0.09, LEFT, mph=3, steps=FADE_STEPS, actual=0.0)
    run(a, 0.0, LEFT, mph=3, pressed=True, torque=self.AGAINST_LEFT, actual=0.005)
    assert not a.hold_released
    assert run(a, -0.03, LEFT, mph=3, actual=0.0) == pytest.approx(-0.09 * 0.9, rel=0.01)

  def test_at_5_mph_releases(self):
    a = self.hold()
    run(a, 0.09, RIGHT, mph=6, steps=FADE_STEPS, actual=0.0)
    run(a, 0.0, RIGHT, mph=6, pressed=True, torque=self.AGAINST_RIGHT, actual=0.0)
    assert a.hold_released

  def test_late_in_the_turn_releases(self):
    a = self.hold()
    k, v = 0.1, 3 * CV.MPH_TO_MS
    run(a, k, RIGHT, mph=3, steps=int(math.radians(25) / (k * v * DT_CTRL)))  # turned 25 degrees
    run(a, 0.05, RIGHT, mph=3, pressed=True, torque=self.AGAINST_RIGHT, actual=0.05)
    assert a.hold_released

  def test_straightening_doesnt_add_to_the_turn_left(self):
    a = self.hold()
    run(a, 0.09, RIGHT, mph=3, steps=FADE_STEPS, actual=0.0)
    run(a, 0.0, RIGHT, mph=3, steps=50, pressed=True, torque=self.AGAINST_RIGHT, actual=-0.05)
    assert a.heading == 0.0

  def test_steering_into_the_turn_without_hold_adds_still_releases(self):
    a = self.hold()
    run(a, 0.09, RIGHT, mph=3, steps=FADE_STEPS, actual=0.0)
    run(a, 0.0, RIGHT, mph=3, pressed=True, torque=self.INTO_RIGHT, actual=0.0)
    assert a.hold_released


class TestTurnLead:
  def lead(self, **values):
    return FmTurnAssist(FakeParams(**({PARAM_LEAD: True} | values)))

  def test_off_by_default(self):
    a = FmTurnAssist(FakeParams())
    assert run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(10.0), actual=0.0) == 0.0

  def test_path_geometry_sign(self):
    p = path(10.0, start=0.0)
    assert plan_circle_curvature(p.position.x, p.position.y, 7.0) == pytest.approx(0.1, rel=0.05)  # right is positive
    q = path(10.0, start=0.0, side=-1)
    assert plan_circle_curvature(q.position.x, q.position.y, 7.0) == pytest.approx(-0.1, rel=0.05)

  def test_turns_in_when_model_is_late(self):
    a = self.lead()
    out = run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    assert 0.02 < out <= 1 / MIN_RADIUS_DEFAULT + 1e-9

  def test_left_turn(self):
    a = self.lead()
    assert run(a, 0.0, LEFT, steps=FADE_STEPS, model=path(15.0, side=-1), actual=0.0) < -0.02

  def test_never_toward_the_other_side(self):
    a = self.lead()
    assert run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0, side=-1), actual=0.0) == 0.0

  @pytest.mark.parametrize("setting,radius", [(MIN_RADIUS_DEFAULT, MIN_RADIUS_DEFAULT), (8.0, 8.0), (3.0, 8.0), (40.0, 20.0)])
  def test_radius_cap(self, setting, radius):
    a = self.lead(**{PARAM_MIN_RADIUS: setting})
    out = run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(5.0, start=0.0), actual=0.0)
    assert out == pytest.approx(1 / radius)

  def test_hands_back_as_truck_catches_up(self):
    a = self.lead()
    half = run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    assert run(a, 0.0, RIGHT, model=path(15.0), actual=half) == 0.0  # turning as tightly as the path asks: model again

  def test_never_reduces_model_request(self):
    a = self.lead()
    assert run(a, 0.09, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0) == 0.09

  @pytest.mark.parametrize("mph,active", [(5, False), (9, True), (13, True), (17, False)])
  def test_speed_band(self, mph, active):
    a = self.lead()
    out = run(a, 0.0, RIGHT, mph=mph, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    assert (out > 0.0) == active

  @pytest.mark.parametrize("setting,mph,active", [(7.0, 6, False), (3.0, 2, False), (3.0, 4, True), (2.0, 3, True),
                                                  (1.0, 1.5, False), (10.0, 6, False)])
  def test_min_speed_setting(self, setting, mph, active):
    a = self.lead(**{PARAM_LEAD_MIN_SPEED: setting})  # clipped to 2..7 mph
    out = run(a, 0.0, RIGHT, mph=mph, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    assert (out > 0.0) == active

  def test_full_strength_about_2_mph_above_min(self):
    a = self.lead(**{PARAM_LEAD_MIN_SPEED: 3.0})
    full = run(a, 0.0, RIGHT, mph=9, steps=FADE_STEPS, model=path(15.0, start=0.0), actual=0.0)  # a circle: same curve at any distance
    b = self.lead(**{PARAM_LEAD_MIN_SPEED: 3.0})
    assert run(b, 0.0, RIGHT, mph=5.3, steps=FADE_STEPS, model=path(15.0, start=0.0), actual=0.0) == pytest.approx(full, rel=0.05)

  def test_not_when_stopping_short(self):
    a = self.lead()
    assert run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0, a_ego=-3.0) == 0.0

  def test_not_when_model_steers_away(self):
    a = self.lead()
    assert run(a, -0.01, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0) == -0.01

  def test_not_during_lane_change(self):
    a = self.lead()
    assert run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0, lane_changing=True) == 0.0

  def test_no_turn_no_lead(self):
    a = self.lead()
    assert run(a, 0.0, NONE, steps=FADE_STEPS, model=path(15.0), actual=0.0) == 0.0

  def test_no_path_no_lead(self):
    a = self.lead()
    assert run(a, 0.0, RIGHT, steps=FADE_STEPS, model=None, actual=0.0) == 0.0
    assert run(a, 0.0, RIGHT, steps=FADE_STEPS, model=SimpleNamespace(), actual=0.0) == 0.0

  def test_hold_keeps_what_lead_asked(self):
    a = self.lead(**{PARAM_HOLD: True})
    asked = run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    assert run(a, 0.0, RIGHT, model=path(None), actual=0.0) == pytest.approx(asked * 0.9, rel=0.02)


class TestTurnNudge:
  @pytest.mark.parametrize("direction, sign", [(LEFT, -1), (RIGHT, 1)])
  def test_deliberate_nudge_adds_bounded_curvature_and_yields(self, direction, sign):
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import PARAM_TURN_NUDGE
    a = FmTurnAssist(FakeParams(**{PARAM_HOLD: True, PARAM_HOLD_DRIVER: True, PARAM_TURN_NUDGE: True}))
    model = path(12, start=0.0, side=sign)
    opts = dict(mph=4, actual=0.0, model=model)
    assert run(a, 0.0, direction, steps=FADE_STEPS, **opts) == 0.0
    assert run(a, 0.0, direction, pressed=True, torque=-sign, **opts) == 0.0
    assert run(a, 0.0, direction, steps=15, pressed=True, torque=-sign, **opts) == pytest.approx(sign * 0.01 * 4 / 6)
    assert a.peak == 0.0
    assert sign * run(a, 0.0, direction, **opts) > 0.0
    assert run(a, 0.0, direction, pressed=True, torque=sign, **opts) == 0.0
    assert run(a, 0.0, direction, **opts) == 0.0

  def test_nudge_does_not_turn_in_when_stopping(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import PARAM_TURN_NUDGE
    a = FmTurnAssist(FakeParams(**{PARAM_HOLD: True, PARAM_HOLD_DRIVER: True, PARAM_TURN_NUDGE: True}))
    assert run(a, 0.0, RIGHT, mph=4, actual=0.0, steps=FADE_STEPS, pressed=True, torque=-1.0,
               model=path(12, start=0.0), a_ego=-1.0) == 0.0


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


class TestSmoothTurnStart:
  """Smooth Turn Start: the extra request eases in over about 0.3 s and drops at once; the model's own request is never
  filtered (00000068--1 +24.3 s: a stepped Turn Lead request saturated the controller, which then swung back)."""

  def _boosted(self, phase_in):
    assist = FmTurnAssist(FakeParams(**{PARAM_BOOST: True, PARAM_BOOST_RIGHT: 1.5, PARAM_PHASE_IN: phase_in}))
    run(assist, 0.01, RIGHT, steps=FADE_STEPS + 100)  # fully faded in, steady
    return assist

  def test_off_steps_at_once(self):
    assist = self._boosted(False)
    assert run(assist, 0.05, RIGHT) == pytest.approx(0.075)

  def test_on_eases_a_step_in_over_about_0_3_s(self):
    assist = self._boosted(True)
    assert run(assist, 0.01, RIGHT) == pytest.approx(0.015, abs=1e-5)  # steady before the step
    first = run(assist, 0.05, RIGHT)
    assert 0.05 + 0.005 < first < 0.05 + 0.005 + 0.003  # the model's own step passes; the extra barely moves
    after = run(assist, 0.05, RIGHT, steps=int(0.3 / DT_CTRL) - 1)
    assert after == pytest.approx(0.075, abs=0.025 * 0.12)  # about 90% of the extra
    assert run(assist, 0.05, RIGHT, steps=int(6 * PHASE_IN_TAU / DT_CTRL)) == pytest.approx(0.075, abs=1e-4)

  def test_on_drops_at_once(self):
    assist = self._boosted(True)
    run(assist, 0.05, RIGHT, steps=200)
    assert run(assist, 0.01, RIGHT) == pytest.approx(0.015)

  def test_model_request_is_never_filtered(self):
    assist = FmTurnAssist(FakeParams(**{PARAM_HOLD: True, PARAM_PHASE_IN: True}))
    run(assist, 0.0, RIGHT, steps=FADE_STEPS)
    assert run(assist, 0.04, RIGHT) == pytest.approx(0.04)

  def test_left_turn_is_eased_the_same(self):
    assist = FmTurnAssist(FakeParams(**{PARAM_BOOST: True, PARAM_BOOST_LEFT: 1.5, PARAM_PHASE_IN: True}))
    run(assist, -0.01, LEFT, steps=FADE_STEPS + 100)
    first = run(assist, -0.05, LEFT)
    assert -0.05 - 0.008 < first < -0.05 - 0.005

  def test_new_turn_starts_from_zero(self):
    assist = self._boosted(True)
    run(assist, 0.05, RIGHT, steps=200)
    run(assist, 0.0, NONE, steps=FADE_STEPS)
    assert assist.extra == 0.0

  def test_signal_cancel_fades_from_what_was_applied(self):
    assist = FmTurnAssist(FakeParams(**{PARAM_HOLD: True, PARAM_PHASE_IN: True}))
    run(assist, 0.0, RIGHT, steps=FADE_STEPS)
    run(assist, 0.06, RIGHT, steps=20)  # the hold's peak
    applied = run(assist, 0.0, RIGHT)  # the model lets go: the hold's floor eases in
    assert 0.0 < applied < 0.06 * 0.9
    assert run(assist, 0.0, NONE) <= applied


class TestAdaptiveLookAhead:
  """Adaptive Look-Ahead: Turn Lead reads 0.5 s (gentle) to 0.9 s (tight) ahead instead of a fixed 1.3 s, at least 3 m."""

  def _assist(self, adaptive):
    return FmTurnAssist(FakeParams(**{PARAM_LEAD: True, PARAM_LEAD_MIN_SPEED: 2.0, PARAM_LEAD_ADAPTIVE: adaptive}))

  def test_off_is_the_fixed_1_3_s(self):
    a = self._assist(False)
    v = 13 * CV.MPH_TO_MS
    assert a.lead_distance(path(12, start=2.0), 1, v) == pytest.approx(1.3 * v)

  @pytest.mark.parametrize("radius,seconds", [(60, 0.5), (10, 0.9)])
  def test_on_reads_by_how_tight(self, radius, seconds):
    a = self._assist(True)
    v = 14 * CV.MPH_TO_MS
    assert a.lead_distance(path(radius, start=0.0), 1, v) == pytest.approx(max(seconds * v, 3.0), rel=0.15)

  @pytest.mark.parametrize("radius", [8, 12, 20, 50])
  def test_plan_max_curvature_finds_the_corner(self, radius):
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import plan_max_curvature
    for side in (1, -1):
      assert plan_max_curvature(path(radius, start=6.0, side=side), side, 17.0) == pytest.approx(1 / radius, rel=0.2)
      assert plan_max_curvature(path(radius, start=6.0, side=side), -side, 17.0) == 0.0
    assert plan_max_curvature(path(None), 1, 17.0) == 0.0
    assert plan_max_curvature(path(12, start=20.0), 1, 17.0) == 0.0  # past the window

  def test_on_never_closer_than_3_m(self):
    a = self._assist(True)
    assert a.lead_distance(path(None), 1, 3 * CV.MPH_TO_MS) == pytest.approx(3.0)

  @pytest.mark.parametrize("side,direction", [(1, RIGHT), (-1, LEFT)])
  def test_corner_5_5_m_ahead_waits_with_adaptive(self, side, direction):
    """At 13 mph a fixed 1.3 s (7.5 m) already reads into a corner 5.5 m ahead; 0.9 s (5.2 m) doesn't yet."""
    model = path(12, start=5.5, side=side)
    fixed = run(self._assist(False), 0.0, direction, mph=13, steps=FADE_STEPS, actual=0.0, model=model)
    adaptive = run(self._assist(True), 0.0, direction, mph=13, steps=FADE_STEPS, actual=0.0, model=model)
    assert side * fixed > 0.003
    assert side * adaptive < 0.001  # its probe point is at most 0.5 m into the corner

  @pytest.mark.parametrize("side,direction", [(1, RIGHT), (-1, LEFT)])
  def test_corner_right_ahead_still_leads(self, side, direction):
    model = path(12, start=1.0, side=side)
    adaptive = run(self._assist(True), 0.0, direction, mph=13, steps=FADE_STEPS, actual=0.0, model=model)
    assert side * adaptive > 0.03


class TestLeadStartDistance:
  """Start Distance gate: no Turn Lead until the path has turned 10 degrees toward the turn within the set distance."""

  def _lead(self, model, direction, gate=None, mph=13):
    values = {PARAM_LEAD: True, PARAM_LEAD_MIN_SPEED: 2.0}
    if gate is not None:
      values |= {PARAM_LEAD_GATE: True, PARAM_LEAD_GATE_DIST: gate}
    return run(FmTurnAssist(FakeParams(**values)), 0.0, direction, mph=mph, steps=FADE_STEPS, actual=0.0, model=model)

  @pytest.mark.parametrize("side,direction", [(1, RIGHT), (-1, LEFT)])
  def test_corner_too_far_waits(self, side, direction):
    model = path(12, start=5.5, side=side)  # 10 degree point at about 8.5 m
    assert side * self._lead(model, direction) > 0.003  # gate off: the fixed 1.3 s reads into it
    assert self._lead(model, direction, gate=5.0) == 0.0
    assert side * self._lead(model, direction, gate=10.0) > 0.003

  @pytest.mark.parametrize("side,direction", [(1, RIGHT), (-1, LEFT)])
  def test_corner_close_leads(self, side, direction):
    model = path(12, start=1.0, side=side)  # 10 degree point at about 4 m
    assert self._lead(model, direction, gate=5.0) == pytest.approx(self._lead(model, direction))

  def test_bend_the_other_way_does_not_open_it(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import plan_turn_onset_toward
    assert math.isinf(plan_turn_onset_toward(path(12, start=1.0, side=-1), 1))
    assert 3.0 < plan_turn_onset_toward(path(12, start=1.0, side=1), 1) < 4.5  # 2-3 m into the corner
    assert math.isinf(plan_turn_onset_toward(path(None), 1))

  def test_gate_setting_is_clamped(self):
    assert FmTurnAssist(FakeParams(**{PARAM_LEAD_GATE_DIST: 50.0})).lead_gate_dist == 10.0
    assert FmTurnAssist(FakeParams(**{PARAM_LEAD_GATE_DIST: "x"})).lead_gate_dist == 5.0


class TestHoldReleaseDebounce:
  """Ignore Brief Hand Blips: a press against the turn releases Turn Hold only once it lasts 0.25 s or reaches 0.5 Nm."""
  AGAINST_RIGHT = 0.14  # Nm, positive (left) is against a right turn: 00000070--77 +33.78 s

  def hold(self, debounce=True, **values):
    a = FmTurnAssist(FakeParams(**({PARAM_HOLD: True, PARAM_HOLD_DEBOUNCE: debounce} | values)))
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    return a

  def test_one_frame_blip_keeps_the_hold(self):
    a = self.hold()
    run(a, 0.03, RIGHT, mph=12.7, pressed=True, torque=self.AGAINST_RIGHT, actual=0.005)
    assert not a.hold_released
    assert run(a, 0.03, RIGHT, mph=12.7, actual=0.005) == pytest.approx(0.081, rel=0.02)

  def test_off_releases_on_one_frame(self):
    a = self.hold(debounce=False)
    run(a, 0.03, RIGHT, mph=12.7, pressed=True, torque=self.AGAINST_RIGHT, actual=0.005)
    assert a.hold_released

  def test_sustained_press_releases(self):
    a = self.hold()
    steps = int(round(HOLD_AGAINST_TIME / DT_CTRL))
    run(a, 0.03, RIGHT, steps=steps - 2, pressed=True, torque=self.AGAINST_RIGHT, actual=0.005)
    assert not a.hold_released
    run(a, 0.03, RIGHT, steps=3, pressed=True, torque=self.AGAINST_RIGHT, actual=0.005)
    assert a.hold_released
    assert run(a, 0.03, RIGHT, actual=0.005) == 0.03

  @pytest.mark.parametrize("direction, torque", [(RIGHT, 0.6), (LEFT, -0.6)])
  def test_firm_press_releases_at_once(self, direction, torque):
    sign = 1 if direction == RIGHT else -1
    a = FmTurnAssist(FakeParams(**{PARAM_HOLD: True, PARAM_HOLD_DEBOUNCE: True}))
    run(a, sign * 0.09, direction, steps=FADE_STEPS, actual=sign * 0.001)
    run(a, sign * 0.03, direction, pressed=True, torque=torque, actual=sign * 0.005)
    assert a.hold_released

  def test_blips_must_be_continuous(self):
    a = self.hold()
    for _ in range(10):  # 20 ms pressed, 10 ms not, for 0.3 s
      run(a, 0.03, RIGHT, steps=2, pressed=True, torque=self.AGAINST_RIGHT, actual=0.005)
      run(a, 0.03, RIGHT, actual=0.005)
    assert not a.hold_released

  def test_into_turn_with_driver_adds_unchanged(self):
    a = self.hold(**{PARAM_HOLD_DRIVER: True})
    assert run(a, 0.03, RIGHT, steps=50, pressed=True, torque=-1.5, actual=0.12) == pytest.approx(0.12)
    assert not a.hold_released

