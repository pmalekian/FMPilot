"""FM Pilot: Turn Lead, Turn Hold, Smooth Turn Start, Finish Turn After Signal Cancels and the Turn Assist log
(fm_turn_assist.py).

Ported from fm-dev-nav-settle ca08aac9 (test_fm_turn_assist.py). Turn Nudge, Hold Through Early Correction, Ignore Brief
Hand Blips, Hold Starts at Turn-In, Hold Only Counts While Steering and Hold Ignores Crawl Spikes are folded into Turn
Hold (always on with it), so the tests that drove them off are gone and the ones that relied on them being off now turn
the truck past 10 degrees, press firmly or press long enough. Turn Lead and its options came with plan PR 11, Finish
Turn After Signal Cancels with plan PR 12 (Carry Only at Turn-In folded, so ca08aac9's test with it off now checks the
folded behavior with the old key stored off). Turn Boost (dropped) tests are not here.
"""
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib import fm_turn_assist
from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import (FmTurnAssist, TurnDirection, PARAM_HOLD, HOLD_MAX_HEADING,
                                                                         FADE_TIME, PARAM_MIN_RADIUS, MIN_RADIUS_DEFAULT,
                                                                         PARAM_HOLD_PATH, PARAM_HOLD_DRIVER, PARAM_PHASE_IN,
                                                                         PHASE_IN_TAU, HOLD_AGAINST_TIME, PARAM_HOLD_FOLLOW_EXIT,
                                                                         PARAM_RESTART_AFTER_STOP, FOLDED_KEYS, turn_sign,
                                                                         PARAM_LEAD, PARAM_LEAD_MIN_SPEED, PARAM_LEAD_ADAPTIVE,
                                                                         PARAM_LEAD_GATE, PARAM_LEAD_GATE_DIST, PARAM_LEAD_YIELD,
                                                                         plan_circle_curvature, PARAM_CARRY, CARRY_MAX_TIME,
                                                                         HOLD_AGAINST_TORQUE)

LEFT, RIGHT, NONE = TurnDirection.turnLeft, TurnDirection.turnRight, TurnDirection.none
FADE_STEPS = int(FADE_TIME / DT_CTRL) + 2
SETTING_KEYS = (PARAM_HOLD, PARAM_HOLD_PATH, PARAM_HOLD_DRIVER, PARAM_HOLD_FOLLOW_EXIT, PARAM_RESTART_AFTER_STOP, PARAM_PHASE_IN,
                PARAM_CARRY)
CARRY_TURN_IN_KEY = "FM_TurnCarryTurnInOnly"  # Carry Only at Turn-In, folded
LEAD_TOGGLES = (PARAM_LEAD, PARAM_LEAD_ADAPTIVE, PARAM_LEAD_GATE, PARAM_LEAD_YIELD)


class FakeParams:
  def __init__(self, **values):
    self.values = {PARAM_HOLD: False, PARAM_HOLD_PATH: False, PARAM_HOLD_DRIVER: False, PARAM_HOLD_FOLLOW_EXIT: False,
                   PARAM_RESTART_AFTER_STOP: False, PARAM_PHASE_IN: False, PARAM_MIN_RADIUS: MIN_RADIUS_DEFAULT,
                   PARAM_LEAD: False, PARAM_LEAD_MIN_SPEED: 7.0, PARAM_LEAD_ADAPTIVE: False, PARAM_LEAD_GATE: False,
                   PARAM_LEAD_GATE_DIST: 5.0, PARAM_LEAD_YIELD: False, PARAM_CARRY: False} | values
    self.reads = set()

  def get_bool(self, key):
    self.reads.add(key)
    return bool(self.values.get(key))

  def get(self, key, return_default=False):
    self.reads.add(key)
    return self.values.get(key)


def hold(**values):
  return FmTurnAssist(FakeParams(**({PARAM_HOLD: True} | values)))


def run(assist, k, direction, mph=10, steps=1, actual=None, pressed=False, lat=True, model=None, a_ego=0.0, lane_changing=False,
        torque=0.0, settle_active=False):
  out = k
  for _ in range(steps):
    out = assist.update(k, direction, mph * CV.MPH_TO_MS, k if actual is None else actual, pressed, lat, model_v2=model,
                        a_ego=a_ego, lane_changing=lane_changing, steering_torque=torque, settle_active=settle_active)
  return out


def into_turn(a, sign=1):
  """0.1 1/m at 10 mph for 0.5 s: about 13 deg turned, past Hold Starts at Turn-In's 10 deg."""
  direction = RIGHT if sign > 0 else LEFT
  run(a, sign * 0.1, direction, steps=50, actual=sign * 0.1)
  assert math.degrees(a.turned) > 10


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


def turn_assist_msg():
  return custom.FmControls.new_message().turnAssist


class TestSignConvention:
  def test_right_turn_is_positive_curvature(self):
    # desiredCurvature is positive to the right (controlsd: curvature = -VM.calc_curvature(left-positive angle))
    assert turn_sign(RIGHT) == 1
    assert turn_sign(LEFT) == -1
    assert turn_sign(NONE) == 0
    assert turn_sign(None) == 0

  def test_hold_never_pulls_away_from_the_turn(self):
    a = hold()
    run(a, -0.09, LEFT, steps=FADE_STEPS, actual=-0.001)
    assert run(a, -0.03, LEFT, actual=-0.001) < -0.03  # holds the left turn tighter, never to the right


class TestOff:
  def test_off_by_default_changes_nothing(self):
    a = FmTurnAssist(FakeParams())
    assert not a.enabled
    rng = np.random.default_rng(1)
    for _ in range(2000):
      k = float(rng.uniform(-0.15, 0.15))
      assert run(a, k, RIGHT if k > 0 else LEFT, mph=float(rng.uniform(0, 20)), actual=k * 0.5, pressed=bool(rng.random() < 0.2),
                 torque=float(rng.uniform(-2, 2)), model=path(10.0, start=0.0)) == k

  @pytest.mark.parametrize("key", SETTING_KEYS[1:])
  def test_options_need_turn_hold(self, key):
    """Hold Path Source, Hold Adds Your Steering, Follows the Model's Exit, Stop Before Turning Starts Over, Smooth
    Turn Start and Finish Turn After Signal Cancels do nothing without Turn Hold (or Turn Lead)."""
    a = FmTurnAssist(FakeParams(**{key: True}))
    for k, d in [(0.0, RIGHT), (0.09, RIGHT), (0.02, RIGHT), (0.01, NONE)] * 30:
      assert run(a, k, d, mph=1, actual=0.06, pressed=True, torque=-1.5, model=path(10.0, start=0.0)) == k


class TestTurnHold:
  def test_holds_when_model_lets_go(self):
    a = hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    out = run(a, 0.03, RIGHT, actual=0.001)  # model eases to a 33 m radius mid-turn
    assert out == pytest.approx(0.09 * 0.9, rel=0.01)

  @pytest.mark.parametrize("direction, sign", [(RIGHT, 1), (LEFT, -1)])
  def test_signal_cancellation_fades_active_hold(self, direction, sign):
    a = hold()
    run(a, sign * 0.09, direction, steps=FADE_STEPS, actual=0.001)
    held = run(a, sign * 0.03, direction, actual=0.001)
    first = run(a, sign * 0.03, NONE)
    middle = run(a, sign * 0.03, NONE, steps=FADE_STEPS // 2)
    assert sign * held > sign * first > sign * middle > 0.03
    assert run(a, sign * 0.03, NONE, steps=FADE_STEPS) == sign * 0.03

  @pytest.mark.parametrize("exit_options", [{"pressed": True}, {"lat": False}, {"lane_changing": True}, {"mph": 21}])
  def test_cancellation_fade_yields_to_exit_conditions(self, exit_options):
    a = hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.03, RIGHT)
    assert run(a, 0.03, NONE) > 0.03
    assert run(a, 0.03, NONE, **exit_options) == 0.03
    assert run(a, 0.03, NONE) == 0.03

  def test_cancelled_hold_does_not_carry_into_new_turn(self):
    a = hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.03, RIGHT)
    assert run(a, 0.03, NONE) > 0.03
    assert run(a, 0.03, RIGHT) == 0.03

  def test_cancellation_does_not_revive_released_hold(self):
    a = hold()
    into_turn(a)
    run(a, 0.03, RIGHT, actual=0.1, pressed=True, torque=1.6)  # a firm push against the right turn
    assert a.hold_released
    assert run(a, 0.03, NONE) == 0.03

  def test_floor_decays(self):
    a = hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    out = run(a, 0.03, RIGHT, steps=int(3 / DT_CTRL), actual=0.001)
    assert abs(out) < 0.09 * 0.9 - 0.09 * 0.15 * 2.5

  def test_follows_model_when_it_asks_more(self):
    a = hold()
    run(a, 0.05, RIGHT, steps=FADE_STEPS, actual=0.001)
    assert run(a, 0.10, RIGHT, actual=0.001) == 0.10

  def test_releases_after_heading(self):
    a = hold()
    k, v = 0.1, 10 * CV.MPH_TO_MS
    frames = int(HOLD_MAX_HEADING / (abs(k) * v * DT_CTRL)) + 5
    run(a, k, RIGHT, steps=frames)
    assert a.hold_released and a.release_reason == fm_turn_assist.RELEASE_TURNED
    assert run(a, 0.02, RIGHT, actual=0.02) == 0.02

  def test_releases_when_driver_steers_against_the_turn(self):
    a = hold()
    into_turn(a)
    assert run(a, 0.03, RIGHT, actual=0.1, pressed=True, torque=1.6) == 0.03
    assert run(a, 0.03, RIGHT, actual=0.1) == 0.03  # stays released for this turn

  def test_new_turn_resets(self):
    a = hold()
    into_turn(a)
    run(a, 0.03, RIGHT, actual=0.1, pressed=True, torque=1.6)
    assert a.hold_released
    run(a, 0.0, NONE)
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    assert run(a, 0.03, RIGHT, actual=0.001) == pytest.approx(0.081, rel=0.01)

  def test_waiting_stopped_does_not_use_up_the_hold(self):
    a = hold()
    run(a, 0.0, RIGHT, mph=0, steps=int(20 / DT_CTRL), actual=0.0)  # 20 s waiting at the corner
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    assert run(a, 0.03, RIGHT, actual=0.001) == pytest.approx(0.081, rel=0.01)

  def test_releases_after_8_s_of_turning(self):
    a = hold()
    run(a, 0.05, RIGHT, steps=int(14 / DT_CTRL), actual=0.004)  # 5 deg after ~5 s, then 8 s
    assert a.hold_released and a.release_reason == fm_turn_assist.RELEASE_TIME

  def test_heading_math(self):
    assert math.degrees(HOLD_MAX_HEADING) == pytest.approx(75)


class TestHoldPathSource:
  def hold(self, path_source=True, **values):
    return hold(**({PARAM_HOLD_PATH: path_source} | values))

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

  @pytest.mark.parametrize("value, radius", [(5.0, 8.0), (30.0, 20.0), (None, MIN_RADIUS_DEFAULT), ("x", MIN_RADIUS_DEFAULT),
                                             (float("nan"), MIN_RADIUS_DEFAULT), (15.0, 15.0)])
  def test_radius_setting_is_clamped(self, value, radius):
    assert self.hold(**{PARAM_MIN_RADIUS: value}).max_curvature == pytest.approx(1 / radius)

  def test_holds_path_turn_after_speeding_up(self):
    a = self.hold()
    # long enough at the crawl for Hold Ignores Crawl Spikes' 0.3 s window to see only the faded-in request
    asked = run(a, 0.0, RIGHT, mph=1, steps=FADE_STEPS + 40, model=path(10.0, start=0.0), actual=0.0)
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

  def test_bad_model_ignored(self):
    a = self.hold()
    assert run(a, 0.0, RIGHT, mph=1, steps=FADE_STEPS, model=SimpleNamespace(), actual=0.0) == 0.0


class TestHoldDriverAdds:
  # hand torque is positive to the LEFT; curvature positive to the RIGHT
  INTO_RIGHT, INTO_LEFT = -1.5, 1.5
  PRESS_STEPS = int(round(HOLD_AGAINST_TIME / DT_CTRL)) + 1  # Ignore Brief Hand Blips: a press counts after 0.25 s

  def hold(self, adds=True, **values):
    return hold(**({PARAM_HOLD_DRIVER: adds} | values))

  def test_off_driver_still_releases(self):
    a = self.hold(adds=False)
    run(a, 0.03, RIGHT, steps=FADE_STEPS, actual=0.03)
    assert run(a, 0.03, RIGHT, steps=self.PRESS_STEPS, pressed=True, torque=self.INTO_RIGHT, actual=0.06) == 0.03
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
    into_turn(a)
    assert run(a, 0.03, RIGHT, pressed=True, torque=1.6, actual=0.06) == 0.03
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
    run(a, 0.03, RIGHT, steps=self.PRESS_STEPS, pressed=True, torque=self.INTO_RIGHT, actual=0.0)
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


class TestHoldEarlyCorrection:
  """Hold Through Early Correction (folded, always on with Turn Hold)."""
  # hand torque is positive to the LEFT; curvature positive to the RIGHT
  AGAINST_RIGHT, AGAINST_LEFT, INTO_RIGHT = 1.5, -1.5, -1.5

  def test_straightening_at_the_start_keeps_the_hold(self):
    a = hold()
    run(a, 0.09, RIGHT, mph=3, steps=FADE_STEPS, actual=0.0)
    # while you steer, the model's request passes through
    assert run(a, 0.0, RIGHT, mph=3, steps=20, pressed=True, torque=self.AGAINST_RIGHT, actual=-0.005) == 0.0
    assert not a.hold_released and a.log_early
    # let go: the hold carries on from the same peak, its fade paused while you steered
    assert run(a, 0.03, RIGHT, mph=3, actual=0.0) == pytest.approx(0.09 * 0.9, rel=0.01)

  def test_left_turn(self):
    a = hold()
    run(a, -0.09, LEFT, mph=3, steps=FADE_STEPS, actual=0.0)
    run(a, 0.0, LEFT, mph=3, pressed=True, torque=self.AGAINST_LEFT, actual=0.005)
    assert not a.hold_released
    assert run(a, -0.03, LEFT, mph=3, actual=0.0) == pytest.approx(-0.09 * 0.9, rel=0.01)

  def test_above_5_mph_past_turn_in_releases(self):
    a = hold()
    run(a, 0.1, RIGHT, mph=6, steps=80, actual=0.1)  # about 12 deg turned
    run(a, 0.0, RIGHT, mph=6, pressed=True, torque=self.AGAINST_RIGHT, actual=0.1)
    assert a.hold_released

  def test_late_in_the_turn_releases(self):
    a = hold()
    k, v = 0.1, 3 * CV.MPH_TO_MS
    run(a, k, RIGHT, mph=3, steps=int(math.radians(25) / (k * v * DT_CTRL)))  # turned 25 degrees
    run(a, 0.05, RIGHT, mph=3, pressed=True, torque=self.AGAINST_RIGHT, actual=0.05)
    assert a.hold_released

  def test_straightening_doesnt_add_to_the_turn_left(self):
    a = hold()
    run(a, 0.09, RIGHT, mph=3, steps=FADE_STEPS, actual=0.0)
    run(a, 0.0, RIGHT, mph=3, steps=50, pressed=True, torque=self.AGAINST_RIGHT, actual=-0.05)
    assert a.heading == 0.0

  def test_steering_into_the_turn_without_hold_adds_still_releases(self):
    a = hold()
    run(a, 0.09, RIGHT, mph=3, steps=FADE_STEPS, actual=0.0)
    run(a, 0.0, RIGHT, mph=3, steps=int(round(HOLD_AGAINST_TIME / DT_CTRL)) + 1, pressed=True, torque=self.INTO_RIGHT,
        actual=0.0)
    assert a.hold_released


class TestHoldReleaseDebounce:
  """Ignore Brief Hand Blips (folded): a press against the turn releases Turn Hold only once it lasts 0.25 s or reaches
  1.5 Nm (with Hold Starts at Turn-In, also folded, the firm line is 1.5 Nm, not 0.5)."""
  AGAINST_RIGHT = 0.14  # Nm, positive (left) is against a right turn: 00000070--77 +33.78 s

  def test_one_frame_blip_keeps_the_hold(self):
    a = hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.03, RIGHT, mph=12.7, pressed=True, torque=self.AGAINST_RIGHT, actual=0.005)
    assert not a.hold_released
    assert run(a, 0.03, RIGHT, mph=12.7, actual=0.005) == pytest.approx(0.081, rel=0.02)

  def test_one_frame_blip_past_turn_in_keeps_the_hold(self):
    a = hold()
    into_turn(a)
    run(a, 0.03, RIGHT, mph=12.7, pressed=True, torque=self.AGAINST_RIGHT, actual=0.1)
    assert not a.hold_released

  def test_sustained_press_releases(self):
    a = hold()
    into_turn(a)
    steps = int(round(HOLD_AGAINST_TIME / DT_CTRL))
    run(a, 0.03, RIGHT, steps=steps - 2, pressed=True, torque=self.AGAINST_RIGHT, actual=0.1)
    assert not a.hold_released
    run(a, 0.03, RIGHT, steps=3, pressed=True, torque=self.AGAINST_RIGHT, actual=0.1)
    assert a.hold_released
    assert run(a, 0.03, RIGHT, actual=0.1) == 0.03

  @pytest.mark.parametrize("direction, sign", [(RIGHT, 1), (LEFT, -1)])
  def test_firm_press_releases_at_once(self, direction, sign):
    a = hold()
    into_turn(a, sign)
    run(a, sign * 0.03, direction, pressed=True, torque=sign * 0.6, actual=sign * 0.1)
    assert not a.hold_released  # 0.6 Nm isn't firm
    run(a, sign * 0.03, direction, pressed=True, torque=sign * 1.6, actual=sign * 0.1)
    assert a.hold_released

  def test_blips_must_be_continuous(self):
    a = hold()
    into_turn(a)
    for _ in range(10):  # 20 ms pressed, 10 ms not, for 0.3 s
      run(a, 0.03, RIGHT, steps=2, pressed=True, torque=self.AGAINST_RIGHT, actual=0.1)
      run(a, 0.03, RIGHT, actual=0.1)
    assert not a.hold_released

  def test_into_turn_with_driver_adds_unchanged(self):
    a = hold(**{PARAM_HOLD_DRIVER: True})
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    assert run(a, 0.03, RIGHT, steps=50, pressed=True, torque=-1.5, actual=0.12) == pytest.approx(0.12)
    assert not a.hold_released


class TestHoldFollowsModelExit:
  """Hold Follows the Model's Exit: past 45 deg, a model asking under half the hold's peak fades the hold out."""

  @staticmethod
  def assist(on=True, **extra):
    return hold(**({PARAM_HOLD_FOLLOW_EXIT: on} | extra))

  def turn(self, a, deg):
    # 0.1 1/m at 10 mph: 0.0045 rad per frame
    steps = int(math.radians(deg) / (0.1 * 10 * CV.MPH_TO_MS * DT_CTRL))
    run(a, 0.1, RIGHT, steps=steps, actual=0.1)

  def test_model_lets_go_past_45_deg_fades_out(self):
    a = self.assist()
    self.turn(a, 50)
    first = run(a, 0.03, RIGHT, actual=0.1)
    assert a.hold_released and a.release_reason == fm_turn_assist.RELEASE_MODEL_EXIT
    assert 0.03 < first <= 0.1
    outs = [run(a, 0.03, RIGHT, actual=0.1) for _ in range(FADE_STEPS)]
    assert all(x >= y for x, y in zip(outs, outs[1:], strict=False))  # only fades, never steps up
    assert outs[-1] == pytest.approx(0.03)

  def test_before_45_deg_keeps_the_floor(self):
    a = self.assist()
    self.turn(a, 30)
    out = run(a, 0.03, RIGHT, actual=0.1)
    assert not a.hold_released and out == pytest.approx(0.1 * 0.9, rel=1e-2)

  def test_model_still_turning_keeps_the_floor(self):
    a = self.assist()
    self.turn(a, 50)
    out = run(a, 0.06, RIGHT, actual=0.1)  # 60% of the peak: not letting go yet
    assert not a.hold_released and out > 0.06

  def test_press_into_turn_past_45_does_not_block_model_exit(self):
    a = self.assist(**{PARAM_HOLD_DRIVER: True})
    self.turn(a, 50)
    peak_before = a.peak
    first = run(a, 0.03, RIGHT, actual=0.14, pressed=True, torque=-1.5)  # into a right turn is negative torque
    assert a.hold_released and a.release_reason == fm_turn_assist.RELEASE_MODEL_EXIT
    assert a.log_driver == 0.0 and a.peak == pytest.approx(peak_before)
    assert first <= peak_before + 1e-6  # never latches 0.14
    outs = [first] + [run(a, 0.03, RIGHT, actual=0.14, pressed=True, torque=-1.5) for _ in range(FADE_STEPS)]
    assert all(x >= y for x, y in zip(outs, outs[1:], strict=False))
    assert outs[-1] == pytest.approx(0.03)

  def test_drive_case_right_turn_press_at_53_deg_does_not_latch_and_fades(self):
    """Real drive case: right turn ran wide, model straightens at 50-55 deg, driver presses right at 53 deg."""
    a = self.assist(**{PARAM_HOLD_DRIVER: True})
    self.turn(a, 50)
    assert math.degrees(a.turned) >= 49.0
    peak_at_50 = a.peak
    # 50 -> 53 deg: model still easing toward exit (0.06 >= 0.5 * peak), then straightens to 0.02 at 53 deg
    while math.degrees(a.turned) < 53.0:
      run(a, 0.06, RIGHT, actual=0.08)
    assert not a.hold_released
    # At 53 deg: model asks to straighten (0.02), driver nudges right (torque=-1.5, actual=0.15)
    outs = []
    for _ in range(FADE_STEPS):
      out = run(a, 0.02, RIGHT, actual=0.15, pressed=True, torque=-1.5)
      assert a.log_driver == 0.0
      assert a.peak == pytest.approx(peak_at_50)
      outs.append(out)
    assert a.hold_released and a.release_reason == fm_turn_assist.RELEASE_MODEL_EXIT
    assert outs[0] < peak_at_50  # fades from pre-press applied hold, never jumps to 0.15
    assert all(x >= y for x, y in zip(outs, outs[1:], strict=False))
    assert outs[-1] == pytest.approx(0.02)

  def test_driver_adds_before_45_freezes_at_45_and_holds_until_normal_exit(self):
    """Adds Your Steering before 45 deg latches and stays frozen at 45 deg (not faded there) until normal exit."""
    a = self.assist(**{PARAM_HOLD_DRIVER: True})
    self.turn(a, 25)
    # Before 45 deg: driver winds tighter to 0.14 -> latched
    assert run(a, 0.06, RIGHT, steps=15, actual=0.14, pressed=True, torque=-1.5) == pytest.approx(0.14)
    assert not a.hold_released and a.peak == pytest.approx(0.14)
    # Cross 45 deg with model still turning (0.08 >= 0.5 * 0.14): peak stays frozen at 0.14, not faded at 45 deg
    while math.degrees(a.turned) < 48.0:
      out = run(a, 0.08, RIGHT, actual=0.10)
    assert not a.hold_released and a.peak == pytest.approx(0.14) and out > 0.08
    # Brief nudge tighter past 45 deg (< 0.25 s): doesn't raise peak and doesn't release
    run(a, 0.08, RIGHT, steps=10, actual=0.19, pressed=True, torque=-1.5)
    assert not a.hold_released and a.peak == pytest.approx(0.14) and a.log_driver == 0.0
    # Model straightens (< 0.5 * 0.14): normal model-exit fade ends it
    run(a, 0.03, RIGHT, actual=0.08)
    assert a.hold_released and a.release_reason == fm_turn_assist.RELEASE_MODEL_EXIT
    assert run(a, 0.03, RIGHT, steps=FADE_STEPS, actual=0.08) == pytest.approx(0.03)

  def test_pressure_into_turn_before_45_still_blocks_release_and_past_45_releases(self):
    press_steps = int(round(HOLD_AGAINST_TIME / DT_CTRL)) + 5
    # Before 45 deg: sustained press into the turn still blocks release
    a = self.assist(**{PARAM_HOLD_DRIVER: True})
    self.turn(a, 20)
    run(a, 0.06, RIGHT, steps=press_steps, actual=0.12, pressed=True, torque=-1.5)
    assert math.degrees(a.turned) < 45.0
    assert not a.hold_released and a.peak == pytest.approx(0.12)
    # Past 45 deg: sustained press into the turn (while model isn't exiting) no longer has the "never releases" rule
    b = self.assist(on=False, **{PARAM_HOLD_DRIVER: True})
    self.turn(b, 50)
    peak = b.peak
    run(b, 0.08, RIGHT, steps=press_steps, actual=0.15, pressed=True, torque=-1.5)
    assert b.hold_released and b.release_reason == fm_turn_assist.RELEASE_DRIVER
    assert b.peak == pytest.approx(peak)  # never raised past 45 deg
    # Ordinary Turn Hold (Adds Your Steering off): brief correction past 45 deg keeps the assist alive
    c = self.assist(on=False, **{PARAM_HOLD_DRIVER: False})
    self.turn(c, 50)
    run(c, 0.06, RIGHT, steps=10, actual=0.10, pressed=True, torque=-1.5)
    assert not c.hold_released

  def test_off_holds_to_75_deg(self):
    a = self.assist(on=False)
    self.turn(a, 50)
    out = run(a, 0.03, RIGHT, steps=FADE_STEPS, actual=0.1)
    assert not a.hold_released and out > 0.05

  def test_signal_cancel_mid_fade_does_not_restart_it(self):
    a = self.assist()
    self.turn(a, 50)
    run(a, 0.03, RIGHT, steps=FADE_STEPS // 2, actual=0.1)
    mid = run(a, 0.03, RIGHT, actual=0.1)
    after = run(a, 0.03, NONE, actual=0.1)
    assert after <= mid + 1e-9
    assert run(a, 0.03, NONE, steps=FADE_STEPS, actual=0.1) == pytest.approx(0.03)

  def test_logged(self):
    a = self.assist()
    self.turn(a, 50)
    run(a, 0.03, RIGHT, steps=3, actual=0.1)
    m = turn_assist_msg()
    a.fill_log(m)
    assert str(m.releaseReason) == "modelExit" and m.exitFade > 0.0 and not m.holdActive


class TestSettleStepsAside:
  """M-3: once fmModel shows Settle After Turn / Early Right Push active for the current turn, Turn Hold ends its floor
  with a 0.5 s fade and does not restart for that turn, even while the blinker stays on and Hold Follows the Model's Exit
  is off."""

  @pytest.mark.parametrize("direction, sign, post_k", [(LEFT, -1, 0.005), (RIGHT, 1, 0.02)])
  def test_early_right_push_with_blinker_on_fades_and_never_restarts(self, direction, sign, post_k):
    a = hold(**{PARAM_HOLD_FOLLOW_EXIT: False})
    steps = int(math.radians(50) / (0.1 * 10 * CV.MPH_TO_MS * DT_CTRL))
    run(a, sign * 0.1, direction, steps=steps, actual=sign * 0.1)
    assert not a.hold_released and a.peak == pytest.approx(0.1)
    # Early Right Push activates while the blinker is still on (left turn: +0.005 push right; right turn: +0.02)
    first = run(a, post_k, direction, actual=sign * 0.01, settle_active=True)
    assert a.hold_released and a.release_reason == fm_turn_assist.RELEASE_MODEL_EXIT
    assert sign * first > sign * post_k  # 0.5 s fade starts from applied hold
    outs = [first] + [run(a, post_k, direction, actual=sign * 0.01, settle_active=True) for _ in range(FADE_STEPS)]
    assert all(sign * x >= sign * y - 1e-12 for x, y in zip(outs, outs[1:], strict=False))
    assert outs[-1] == pytest.approx(post_k)
    # Settle finishes while blinker is still on: Turn Hold must NOT restart for this turn
    assert run(a, post_k, direction, steps=20, actual=sign * 0.01, settle_active=False) == pytest.approx(post_k)
    assert a.hold_released
    # A new turn starts clean
    run(a, 0.0, NONE, steps=FADE_STEPS)
    run(a, sign * 0.09, direction, steps=FADE_STEPS, actual=sign * 0.01)
    assert not a.hold_released and sign * run(a, sign * 0.02, direction, actual=sign * 0.01) > 0.06

  def test_stale_settle_from_previous_turn_does_not_release_new_turn(self):
    a = hold()
    # Leftover settle from Turn 1 on the first 5 frames of Turn 2 must not kill Turn 2's hold
    run(a, 0.09, RIGHT, steps=5, actual=0.01, settle_active=True)
    assert not a.hold_released and not a.settle_armed
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.01, settle_active=False)
    assert not a.hold_released and a.settle_armed
    # Later in Turn 2, Turn 2's own Settle activates -> now Turn Hold steps aside
    run(a, 0.02, RIGHT, actual=0.01, settle_active=True)
    assert a.hold_released and a.release_reason == fm_turn_assist.RELEASE_MODEL_EXIT

  @pytest.mark.parametrize("direction, sign, post_k", [(LEFT, -1, 0.005), (RIGHT, 1, 0.02)])
  @pytest.mark.parametrize("phase, settle_k", [("push", 0.005), ("centering", 0.003), ("idle", 0.003)])
  def test_fm_lateral_reads_settle_from_fm_model(self, direction, sign, post_k, phase, settle_k):
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_lateral import FmLateral

    class SM(dict):
      def __init__(self, turn_dir, fm_model):
        super().__init__(modelDataV2SP=SimpleNamespace(laneTurnDirection=turn_dir), fmModel=fm_model)
        self.seen = {'modelDataV2SP': True, 'fmModel': True}

    params = FakeParams(**{PARAM_HOLD: True})
    params.values.update({"FM_LaneKeepMode": 0, "IsRhdDetected": False})
    lat = FmLateral(SimpleNamespace(brand='rivian'), params)
    lat.lane_guard.update = lambda *a, **k: 0.0
    CS = SimpleNamespace(vEgo=10 * CV.MPH_TO_MS, steeringPressed=False,
                         leftBlinker=(sign < 0), rightBlinker=(sign > 0), steeringTorque=0.0, aEgo=0.0)
    idle_fm = custom.FmModel.new_message()
    active_fm = custom.FmModel.new_message()
    active_fm.turnSettlePhase = phase
    active_fm.turnSettle = settle_k

    steps = int(math.radians(50) / (0.1 * 10 * CV.MPH_TO_MS * DT_CTRL))
    for _ in range(steps):
      lat.update(sign * 0.1, SimpleNamespace(), CS, True, False, sign * 0.1, SM(direction, idle_fm))
    assert not lat.turn_assist.hold_released
    out = post_k
    for _ in range(FADE_STEPS + 1):
      out = lat.update(post_k, SimpleNamespace(), CS, True, False, sign * 0.01, SM(direction, active_fm))
    assert lat.turn_assist.hold_released and lat.turn_assist.release_reason == fm_turn_assist.RELEASE_MODEL_EXIT
    assert out == pytest.approx(post_k)


class TestHoldStartsAtTurnIn:
  """Hold Starts at Turn-In (folded): Turn Hold isn't used up before the truck turns."""

  def test_clock_waits_for_the_turn(self):
    a = hold()
    run(a, 0.0, RIGHT, steps=int(10.0 / DT_CTRL), actual=0.0)  # 10 s of approach with the blinker on
    assert not a.hold_released and a.turn_time == 0.0
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.02)
    assert run(a, 0.03, RIGHT, actual=0.02) > 0.07  # the hold is there for the turn

  @pytest.mark.parametrize("torque,steps", [(0.62, 5), (1.01, 1), (2.0, 100)])
  def test_press_against_before_10_deg_passes_through(self, torque, steps):
    a = hold()
    run(a, 0.09, RIGHT, mph=15, steps=FADE_STEPS, actual=0.001)
    out = run(a, 0.03, RIGHT, mph=15, steps=steps, actual=0.001, pressed=True, torque=torque)  # left: against a right turn
    assert not a.hold_released
    assert out == pytest.approx(0.03)  # your steering passes through
    assert run(a, 0.03, RIGHT, mph=15, actual=0.001) > 0.07  # and the hold carries on

  def test_long_press_against_before_10_deg_still_releases(self):
    a = hold()
    run(a, 0.09, RIGHT, mph=15, steps=FADE_STEPS, actual=0.001)
    run(a, 0.03, RIGHT, mph=15, steps=int(1.5 / DT_CTRL) + 2, actual=0.001, pressed=True, torque=1.0)
    assert a.hold_released and a.release_reason == fm_turn_assist.RELEASE_DRIVER  # not turning after all

  def test_after_10_deg_brief_press_under_firm_line_ignored(self):
    a = hold()
    into_turn(a)
    run(a, 0.03, RIGHT, actual=0.1, pressed=True, torque=1.0)
    assert not a.hold_released

  def test_after_10_deg_firm_press_releases_at_once(self):
    a = hold()
    into_turn(a)
    run(a, 0.03, RIGHT, actual=0.1, pressed=True, torque=1.6)
    assert a.hold_released and a.release_reason == fm_turn_assist.RELEASE_DRIVER

  def test_after_10_deg_lasting_press_releases(self):
    a = hold()
    into_turn(a)
    run(a, 0.03, RIGHT, steps=int(HOLD_AGAINST_TIME / DT_CTRL) + 1, actual=0.1, pressed=True, torque=0.6)
    assert a.hold_released

  def test_00000073_8_right_blinker_with_hand_on_wheel(self):
    """--8 +13.44 s: the right blinker's desire arrived while he still pressed 0.62 Nm left at 9 mph."""
    a = hold(**{PARAM_HOLD_DRIVER: True})
    run(a, 0.005, RIGHT, mph=9, steps=2, actual=0.004, pressed=True, torque=0.62)
    run(a, 0.005, RIGHT, mph=9, steps=60, actual=0.004)
    assert not a.hold_released


class TestHoldOnlyCountsWhileSteering:
  """Hold Only Counts While Steering (folded): while openpilot isn't steering, controlsd passes the truck's own curvature
  as the request and Turn Assist takes 0.5 s to fade out. Turn Hold must not take the wheel's angle in that time as the
  turn's peak, nor run its clock or its degrees."""

  def _stop_with_the_wheel_wound_in(self, a):
    run(a, 0.03, RIGHT, mph=3, steps=150)  # rolling up, openpilot steering, the model asks 33 m
    assert a.peak == pytest.approx(0.03)
    run(a, 0.12, RIGHT, mph=0.2, steps=50, actual=0.12, lat=False)  # stopped: the request is now the wheel's own 8 m
    run(a, 0.12, RIGHT, mph=0.0, steps=300, actual=0.12, lat=False)  # waiting
    return [run(a, 0.03, RIGHT, mph=3.4, actual=0.05) for _ in range(100)]  # pulling away, the model asks 33 m again

  @pytest.mark.parametrize("extra", [{}, {PARAM_HOLD_DRIVER: True}, {PARAM_PHASE_IN: True}])
  def test_keeps_the_peak_from_before_the_stop(self, extra):
    a = hold(**extra)
    after = self._stop_with_the_wheel_wound_in(a)
    assert a.peak == pytest.approx(0.03)
    assert all(o == pytest.approx(0.03) for o in after)
    assert not a.hold_released

  def test_adds_nothing_and_counts_nothing_while_not_steering(self):
    a = hold()
    run(a, 0.05, RIGHT, mph=8, steps=100)
    peak, since, turn_time, heading = a.peak, a.since_peak, a.turn_time, a.heading
    for _ in range(20):  # inside the 0.5 s fade: the request (the truck's curve) passes through untouched
      assert run(a, 0.01, RIGHT, mph=8, actual=0.01, lat=False) == 0.01
    assert (a.peak, a.since_peak, a.turn_time, a.heading) == (peak, since, turn_time, heading)
    assert a.log_floor == 0.0 and not a.hold_released

  def test_a_press_against_the_turn_while_not_steering_does_not_release(self):
    a = hold()
    run(a, 0.05, RIGHT, mph=8, steps=100)
    run(a, 0.02, RIGHT, mph=8, steps=30, actual=0.02, lat=False, pressed=True, torque=2.0)
    assert not a.hold_released
    # back to steering: the floor from the turn's own peak is there again
    assert run(a, 0.01, RIGHT, mph=8, steps=FADE_STEPS, actual=0.02) > 0.02

  def test_hold_adds_your_steering_still_takes_your_curve_while_steering(self):
    a = hold(**{PARAM_HOLD_DRIVER: True})
    run(a, 0.03, RIGHT, mph=5, steps=100)
    run(a, 0.03, RIGHT, mph=5, steps=50, actual=0.10, pressed=True, torque=-2.0)  # pressing into the right turn
    assert a.peak == pytest.approx(0.10)


class TestHoldSteadyPeak:
  """Hold Ignores Crawl Spikes (folded): under 5 mph the hold's peak only learns a request kept up for 0.3 s."""

  def _spike(self, a, mph):
    run(a, 0.03, RIGHT, mph=mph, steps=50)
    run(a, 0.15, RIGHT, mph=mph, steps=5)  # 50 ms: one model frame
    run(a, 0.04, RIGHT, mph=mph, steps=50)

  def test_ignores_a_crawl_spike(self):
    a = hold()
    self._spike(a, 3)
    assert a.peak == pytest.approx(0.04)
    assert run(a, 0.02, RIGHT, mph=3) < 0.04  # the floor is 90% of 0.04, not of 0.15

  def test_learns_a_steady_crawl_request(self):
    a = hold()
    run(a, 0.03, RIGHT, mph=3, steps=50)
    run(a, 0.12, RIGHT, mph=3, steps=40)  # 0.4 s
    assert a.peak == pytest.approx(0.12)

  def test_left_turn(self):
    a = hold()
    run(a, -0.03, LEFT, mph=3, steps=50)
    run(a, -0.15, LEFT, mph=3, steps=5)
    run(a, -0.04, LEFT, mph=3, steps=50)
    assert a.peak == pytest.approx(0.04)

  def test_above_5_mph_a_spike_is_the_peak(self):
    a = hold()
    self._spike(a, 7)
    assert a.peak == pytest.approx(0.15)


class TestTurnNudge:
  """Turn Nudge (folded, always on with Turn Hold)."""

  @pytest.mark.parametrize("adds", [True, False])
  @pytest.mark.parametrize("direction, sign", [(LEFT, -1), (RIGHT, 1)])
  def test_deliberate_nudge_adds_bounded_curvature_and_yields(self, direction, sign, adds):
    a = hold(**{PARAM_HOLD_DRIVER: adds})
    model = path(12, start=0.0, side=sign)
    opts = {"mph": 4, "actual": 0.0, "model": model}
    assert run(a, 0.0, direction, steps=FADE_STEPS, **opts) == 0.0
    assert run(a, 0.0, direction, pressed=True, torque=-sign, **opts) == 0.0
    assert run(a, 0.0, direction, steps=15, pressed=True, torque=-sign, **opts) == pytest.approx(sign * 0.01 * 4 / 6)
    assert a.peak == 0.0  # never feeds the hold
    assert sign * run(a, 0.0, direction, **opts) > 0.0
    assert run(a, 0.0, direction, pressed=True, torque=sign, **opts) == 0.0
    assert run(a, 0.0, direction, **opts) == 0.0

  def test_nudge_does_not_turn_in_when_stopping(self):
    a = hold(**{PARAM_HOLD_DRIVER: True})
    assert run(a, 0.0, RIGHT, mph=4, actual=0.0, steps=FADE_STEPS, pressed=True, torque=-1.0,
               model=path(12, start=0.0), a_ego=-1.0) == 0.0

  def test_no_nudge_without_turn_hold(self):
    a = FmTurnAssist(FakeParams())
    assert run(a, 0.0, RIGHT, mph=4, actual=0.0, steps=FADE_STEPS, pressed=True, torque=-1.0, model=path(12, start=0.0)) == 0.0


class TestRestartAfterStop:
  """Stop Before Turning Starts Over (FM_TurnRestartAfterStop): a stop before the truck has turned 10 deg starts the
  turn over (000000af--3 +24.8 s: a creep in a queue set a 0.12 peak that came back on the next creep 7 s later as 0.087
  added to a model asking 0.007)."""

  def _assist(self, on, **extra):
    return hold(**({PARAM_RESTART_AFTER_STOP: on} | extra))

  def _creep_stop_creep(self, a):
    run(a, -0.10, LEFT, mph=2, steps=60, actual=-0.03)  # creep: the model asks for the turn
    run(a, -0.03, LEFT, mph=0.0, steps=500, actual=-0.03, lat=False)  # stopped behind the car ahead
    return run(a, -0.007, LEFT, mph=2, steps=60, actual=-0.03)  # next creep: the model asks nearly straight

  def test_off_carries_the_creeps_peak(self):
    a = self._assist(False)
    assert self._creep_stop_creep(a) < -0.05

  def test_on_starts_over(self):
    a = self._assist(True)
    out = self._creep_stop_creep(a)
    assert out == pytest.approx(-0.007)
    assert a.restarts == 1

  def test_on_keeps_a_turn_paused_past_10_deg(self):
    a = self._assist(True)
    run(a, -0.10, LEFT, mph=4, steps=60, actual=-0.10)
    a.turned = math.radians(15.0)  # well into the turn
    run(a, -0.10, LEFT, mph=0.0, steps=300, actual=-0.10, lat=False)
    assert a.peak == pytest.approx(0.10)
    assert a.restarts == 0

  def test_on_rearms_a_hold_released_on_the_creep(self):
    a = self._assist(True)
    run(a, -0.10, LEFT, mph=2, steps=30, actual=-0.03)
    run(a, -0.10, LEFT, mph=2, steps=40, actual=-0.03, pressed=True, torque=1.0)  # into the left turn, no Hold Adds
    assert a.hold_released
    run(a, -0.03, LEFT, mph=0.0, steps=100, actual=-0.03, lat=False)
    assert not a.hold_released

  def test_slowing_without_stopping_does_not_start_over(self):
    a = self._assist(True)
    run(a, -0.10, LEFT, mph=2, steps=60, actual=-0.03)
    run(a, -0.03, LEFT, mph=1.5, steps=200, actual=-0.03)
    assert a.peak == pytest.approx(0.10)
    assert a.restarts == 0

  def test_logged(self):
    a = self._assist(True)
    self._creep_stop_creep(a)
    m = turn_assist_msg()
    a.fill_log(m)
    assert m.restarts == 1

  @pytest.mark.parametrize("turned_deg,state", [(15.0, "lead_off"), (0.0, "lead_paused")])
  def test_on_rearms_turn_lead_yielded_on_the_creep(self, turned_deg, state):
    """Turn Lead switched off (or paused) by your hands on the creep doesn't carry into the turn."""
    a = FmTurnAssist(FakeParams(**{PARAM_LEAD: True, PARAM_LEAD_YIELD: True, PARAM_RESTART_AFTER_STOP: True}))
    run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    a.turned = math.radians(turned_deg)
    run(a, 0.0, RIGHT, steps=30, model=path(15.0), actual=0.0, pressed=True, torque=1.0)
    assert getattr(a, state)
    a.turned = 0.0  # back under 10 deg (a shortcut: the creep turned back), then a stop
    run(a, 0.0, RIGHT, mph=0.0, steps=5, model=path(15.0), actual=0.0, lat=False)
    assert not a.lead_off and not a.lead_paused and a.restarts == 1

  def test_off_keeps_turn_lead_switched_off(self):
    a = FmTurnAssist(FakeParams(**{PARAM_LEAD: True, PARAM_LEAD_YIELD: True}))
    run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    a.turned = math.radians(15.0)
    run(a, 0.0, RIGHT, steps=30, model=path(15.0), actual=0.0, pressed=True, torque=1.0)
    a.turned = 0.0
    run(a, 0.0, RIGHT, mph=0.0, steps=50, model=path(15.0), actual=0.0, lat=False)
    assert a.lead_off and a.restarts == 0


class TestSmoothTurnStart:
  """Smooth Turn Start (FM_TurnPhaseIn): what Turn Hold adds eases in over about 0.3 s and drops at once; the model's
  own request is never filtered (00000068--1 +24.3 s: a stepped request saturated the controller, which then swung back)."""

  def _held(self, phase_in, sign=1):
    a = hold(**{PARAM_PHASE_IN: phase_in})
    direction = RIGHT if sign > 0 else LEFT
    run(a, sign * 0.06, direction, steps=FADE_STEPS + 50, actual=sign * 0.001)  # fully faded in, peak 0.06
    return a, direction

  def test_off_steps_at_once(self):
    a, _ = self._held(False)
    assert run(a, 0.0, RIGHT, actual=0.001) == pytest.approx(0.06 * (0.9 - 0.15 * DT_CTRL))

  def test_on_eases_a_step_in_over_about_0_3_s(self):
    on, _ = self._held(True)
    off, _ = self._held(False)
    first = run(on, 0.0, RIGHT, actual=0.001)
    floor = run(off, 0.0, RIGHT, actual=0.001)
    assert first == pytest.approx(floor * fm_turn_assist.PHASE_IN_ALPHA)  # the extra barely moves
    for _ in range(int(0.3 / DT_CTRL) - 1):
      after, floor = run(on, 0.0, RIGHT, actual=0.001), run(off, 0.0, RIGHT, actual=0.001)
    assert 0.85 * floor < after < floor  # about 90% of the extra
    for _ in range(int(6 * PHASE_IN_TAU / DT_CTRL)):
      after, floor = run(on, 0.0, RIGHT, actual=0.001), run(off, 0.0, RIGHT, actual=0.001)
    assert after == pytest.approx(floor, rel=0.02)

  def test_on_drops_at_once(self):
    a, _ = self._held(True)
    run(a, 0.0, RIGHT, steps=60, actual=0.001)
    assert run(a, 0.08, RIGHT, actual=0.001) == 0.08  # the model asks more than the floor: nothing extra, at once

  def test_model_request_is_never_filtered(self):
    a = hold(**{PARAM_PHASE_IN: True})
    run(a, 0.0, RIGHT, steps=FADE_STEPS)
    assert run(a, 0.04, RIGHT) == pytest.approx(0.04)

  def test_left_turn_is_eased_the_same(self):
    on, _ = self._held(True, sign=-1)
    off, _ = self._held(False, sign=-1)
    assert run(on, 0.0, LEFT, actual=-0.001) == pytest.approx(run(off, 0.0, LEFT, actual=-0.001) * fm_turn_assist.PHASE_IN_ALPHA)

  def test_new_turn_starts_from_zero(self):
    a, _ = self._held(True)
    run(a, 0.0, RIGHT, steps=60, actual=0.001)
    run(a, 0.0, NONE, steps=FADE_STEPS)
    assert a.extra == 0.0

  def test_signal_cancel_fades_from_what_was_applied(self):
    a = hold(**{PARAM_PHASE_IN: True})
    run(a, 0.0, RIGHT, steps=FADE_STEPS)
    run(a, 0.06, RIGHT, steps=20)  # the hold's peak
    applied = run(a, 0.0, RIGHT)  # the model lets go: the hold's floor eases in
    assert 0.0 < applied < 0.06 * 0.9
    assert run(a, 0.0, NONE) <= applied


class TestTurnAssistLog:
  """fmControls.turnAssist: what each part added and why Turn Hold let go. Logging only."""

  def logged(self, a):
    m = turn_assist_msg()
    a.fill_log(m)
    return m

  def test_idle_logs_nothing(self):
    a = hold()
    run(a, 0.01, NONE)
    m = self.logged(a)
    assert m.sign == 0 and str(m.source) == "none" and not m.holdActive
    assert m.requestIn == pytest.approx(0.01) and m.requestOut == pytest.approx(0.01)

  def test_hold_floor_and_peak_logged(self):
    a = hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    out = run(a, 0.03, RIGHT, actual=0.001)
    m = self.logged(a)
    assert m.sign == 1 and str(m.source) == "desire" and m.holdActive
    assert m.holdPeak == pytest.approx(0.09, rel=1e-5)
    assert m.holdFloor == pytest.approx(out - 0.03, rel=1e-4)
    assert m.requestOut == pytest.approx(out, rel=1e-5) and m.requestIn == pytest.approx(0.03)
    assert m.holdFloorLevel == pytest.approx(0.09 * (0.9 - 0.15 * DT_CTRL), rel=1e-4)  # one frame past the peak

  def test_left_turn_toward_is_positive(self):
    a = hold()
    run(a, -0.09, LEFT, steps=FADE_STEPS, actual=-0.001)
    run(a, -0.03, LEFT, actual=-0.001)
    m = self.logged(a)
    assert m.sign == -1 and m.holdFloor > 0.0 and m.requestOut < -0.03

  def test_driver_adds_path_and_nudge_logged(self):
    b = hold(**{PARAM_HOLD_DRIVER: True})
    run(b, 0.02, RIGHT, steps=FADE_STEPS, actual=0.02)
    out = run(b, 0.02, RIGHT, actual=0.1, pressed=True, torque=-1.5)
    m = self.logged(b)
    assert m.driverAdds == pytest.approx(out - 0.02, rel=1e-4) and m.driverAdds > 0.05
    c = hold(**{PARAM_HOLD_PATH: True})
    out = run(c, 0.0, RIGHT, mph=1, steps=FADE_STEPS, model=path(10.0, start=0.0), actual=0.0)
    assert self.logged(c).holdPath == pytest.approx(out, rel=1e-4) and out > 0.0
    d = hold()
    model = path(12, start=0.0)
    run(d, 0.0, RIGHT, mph=4, steps=FADE_STEPS, actual=0.0, model=model)
    out = run(d, 0.0, RIGHT, mph=4, steps=16, actual=0.0, model=model, pressed=True, torque=-1.0)
    assert self.logged(d).nudge == pytest.approx(out, rel=1e-4) and out > 0.0

  def test_turned_counts_every_turn(self):
    a = hold()
    v = 10 * CV.MPH_TO_MS
    steps = 100
    run(a, 0.05, RIGHT, steps=steps, actual=0.05)
    assert self.logged(a).turned == pytest.approx(math.degrees(0.05 * v * DT_CTRL * steps), rel=1e-3)

  @pytest.mark.parametrize("how,reason", [("press", "driverAgainst"), ("heading", "maxTurned"), ("time", "maxTime")])
  def test_release_reason(self, how, reason):
    a = hold()
    if how == "press":
      into_turn(a)
      run(a, 0.03, RIGHT, actual=0.1, pressed=True, torque=1.6)
    elif how == "heading":
      run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
      run(a, 0.03, RIGHT, steps=400, actual=0.2, mph=10)
    else:
      run(a, 0.05, RIGHT, steps=int(14 / DT_CTRL), actual=0.004)
    m = self.logged(a)
    assert a.hold_released and str(m.releaseReason) == reason and not m.holdActive
    run(a, 0.0, NONE, steps=FADE_STEPS)
    run(a, 0.05, LEFT)
    assert str(self.logged(a).releaseReason) == "none"  # a new turn starts clean

  def test_early_correction_and_against_time_logged(self):
    a = hold()
    run(a, 0.09, RIGHT, mph=3, steps=FADE_STEPS, actual=0.0)
    run(a, 0.0, RIGHT, mph=3, steps=20, pressed=True, torque=1.5, actual=0.0)
    m = self.logged(a)
    assert m.earlyCorrection and m.againstTime == pytest.approx(0.2, abs=1e-6) and m.holdActive

  def test_signal_cancel_fade_logged(self):
    a = hold()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.03, RIGHT, actual=0.001)
    run(a, 0.03, NONE, actual=0.001)
    m = self.logged(a)
    assert str(m.source) == "none" and m.sign == 1 and m.exitFade > 0.0 and not m.holdActive

  def test_smooth_start_cut_logged(self):
    a = hold(**{PARAM_PHASE_IN: True})
    run(a, 0.06, RIGHT, steps=FADE_STEPS + 50, actual=0.001)
    out = run(a, 0.0, RIGHT, actual=0.001)
    m = self.logged(a)
    assert m.holdFloor > 0.0 and m.smoothCut == pytest.approx(m.holdFloor - out, rel=1e-4) and m.smoothCut > 0.0

  def test_turn_lead_fields_stay_zero_with_turn_lead_off(self):
    """With Turn Lead off (its options on), its fields stay at their defaults, whatever happens."""
    rng = np.random.default_rng(7)
    a = hold(**{PARAM_HOLD_DRIVER: True, PARAM_HOLD_PATH: True, PARAM_PHASE_IN: True, PARAM_LEAD_ADAPTIVE: True,
                PARAM_LEAD_GATE: True, PARAM_LEAD_YIELD: True, PARAM_LEAD_MIN_SPEED: 2.0})
    for i in range(3000):
      k = 0.08 * math.sin(i / 50)
      run(a, k, RIGHT if (i // 400) % 2 else LEFT, mph=float(rng.uniform(0, 12)), actual=k * 0.8,
          pressed=bool(rng.random() < 0.1), torque=float(rng.uniform(-2, 2)), model=path(10.0, start=0.0))
      m = self.logged(a)
      assert m.lead == 0.0 and m.leadYield == 0 and str(m.source) != "carry"

  def test_lead_logged(self):
    a = FmTurnAssist(FakeParams(**{PARAM_LEAD: True, PARAM_LEAD_MIN_SPEED: 2.0}))
    model = path(10, start=0.0)
    out = run(a, 0.0, RIGHT, mph=5, steps=FADE_STEPS, actual=0.0, model=model)
    m = self.logged(a)
    assert m.lead == pytest.approx(out, rel=1e-4) and m.lead > 0.0 and not m.holdActive and m.leadYield == 0
    run(a, 0.0, NONE, steps=FADE_STEPS)
    assert self.logged(a).lead == 0.0  # Turn Lead has no cancel fade

  def test_turned_counts_with_turn_lead_only(self):
    a = FmTurnAssist(FakeParams(**{PARAM_LEAD: True}))
    v = 10 * CV.MPH_TO_MS
    run(a, 0.05, RIGHT, steps=100, actual=0.05)
    assert self.logged(a).turned == pytest.approx(math.degrees(0.05 * v * DT_CTRL * 100), rel=1e-3)

  def test_lead_smooth_start_cut_logged(self):
    a = FmTurnAssist(FakeParams(**{PARAM_LEAD: True, PARAM_LEAD_MIN_SPEED: 2.0, PARAM_PHASE_IN: True}))
    run(a, 0.0, RIGHT, mph=5, steps=FADE_STEPS, actual=0.0)  # no model: no lead yet
    run(a, 0.0, RIGHT, mph=5, actual=0.0, model=path(10, start=0.0))
    m = self.logged(a)
    assert m.lead > 0.0 and m.smoothCut > 0.0 and m.requestOut < m.lead

  def test_logging_changes_nothing(self):
    opts = {PARAM_HOLD_DRIVER: True, PARAM_HOLD_PATH: True, PARAM_PHASE_IN: True, PARAM_LEAD: True, PARAM_LEAD_MIN_SPEED: 2.0,
            PARAM_LEAD_YIELD: True}
    a, b = hold(**opts), hold(**opts)
    model = path(10, start=0.0)
    for k, act, d in [(0.0, 0.0, RIGHT), (0.02, 0.05, RIGHT), (0.01, 0.09, RIGHT), (0.01, 0.05, NONE)] * 30:
      x = run(a, k, d, mph=5, actual=act, model=model)
      a.fill_log(turn_assist_msg())
      assert x == run(b, k, d, mph=5, actual=act, model=model)


class TestFolded:
  """Turn Nudge, Hold Through Early Correction, Ignore Brief Hand Blips, Hold Starts at Turn-In, Hold Only Counts While
  Steering and Hold Ignores Crawl Spikes are always on with Turn Hold; their old keys are never read."""
  OLD_OFF = dict.fromkeys(FOLDED_KEYS, False)

  def test_folded_keys_are_the_ones_in_folded_defaults(self):
    assert set(FOLDED_KEYS) == {"FM_TurnNudgeEnabled", "FM_TurnHoldEarlyCorrection", "FM_TurnHoldReleaseDebounce",
                                "FM_TurnHoldStartsAtTurnIn", "FM_TurnHoldSteeringOnly", "FM_TurnHoldSteadyPeak",
                                CARRY_TURN_IN_KEY}
    doc = (Path(__file__).parents[6] / "docs" / "fm-dev-sync" / "FM_FOLDED_DEFAULTS.md").read_text()
    for key in FOLDED_KEYS:
      assert f"`{key}`" in doc

  def test_old_keys_never_read(self):
    params = FakeParams(**({PARAM_HOLD: True, PARAM_HOLD_DRIVER: True, PARAM_HOLD_PATH: True, PARAM_CARRY: True} | self.OLD_OFF))
    a = FmTurnAssist(params)
    for i in range(500):
      run(a, 0.05 * math.sin(i / 40), RIGHT, mph=3, actual=0.03, pressed=i % 7 == 0, torque=1.0, model=path(10.0, start=0.0))
      if i % 100 == 0:
        a.read_params()
    assert params.reads and not params.reads & set(FOLDED_KEYS)
    source = Path(fm_turn_assist.__file__).read_text()
    for key in FOLDED_KEYS:
      assert source.count(f'"{key}"') == 1  # only in FOLDED_KEYS

  def test_folded_behaviors_on_with_the_old_keys_off(self):
    """Each folded behavior acts with its old key stored as off (as a fm-dev-nav-settle setup might have it)."""
    def assist():
      return FmTurnAssist(FakeParams(**({PARAM_HOLD: True} | self.OLD_OFF)))
    # Ignore Brief Hand Blips + Hold Starts at Turn-In: a one-frame 1.0 Nm press past 10 deg doesn't release
    a = assist()
    into_turn(a)
    run(a, 0.03, RIGHT, actual=0.1, pressed=True, torque=1.0)
    assert not a.hold_released
    # Hold Starts at Turn-In: 10 s of approach doesn't spend the 8 s
    a = assist()
    run(a, 0.0, RIGHT, steps=int(10.0 / DT_CTRL), actual=0.0)
    assert not a.hold_released and a.turn_time == 0.0
    # Hold Through Early Correction: straightening under 5 mph passes through
    a = assist()
    run(a, 0.09, RIGHT, mph=3, steps=FADE_STEPS, actual=0.0)
    assert run(a, 0.0, RIGHT, mph=3, steps=20, pressed=True, torque=1.5, actual=-0.005) == 0.0 and not a.hold_released
    # Hold Only Counts While Steering: the wheel's angle while not steering isn't the peak
    a = assist()
    run(a, 0.03, RIGHT, mph=3, steps=150)
    run(a, 0.12, RIGHT, mph=0.2, steps=50, actual=0.12, lat=False)
    assert a.peak == pytest.approx(0.03)
    # Hold Ignores Crawl Spikes
    a = assist()
    run(a, 0.03, RIGHT, mph=3, steps=50)
    run(a, 0.15, RIGHT, mph=3, steps=5)
    assert a.peak == pytest.approx(0.03)
    # Turn Nudge
    a = assist()
    model = path(12, start=0.0)
    run(a, 0.0, RIGHT, mph=4, steps=FADE_STEPS, actual=0.0, model=model)
    assert run(a, 0.0, RIGHT, mph=4, steps=16, actual=0.0, model=model, pressed=True, torque=-1.0) > 0.0


class TestFmLateralWiring:
  class ControlsdSM(dict):
    def __init__(self, direction, seen=True):
      super().__init__(modelDataV2SP=SimpleNamespace(laneTurnDirection=direction))
      self.seen = {'modelDataV2SP': seen}

    def update(self, timeout=None):
      raise AssertionError("controlsd's SubMaster is updated by controlsd, once a step")

  def _lateral(self, **values):
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_lateral import FmLateral
    params = FakeParams(**({PARAM_HOLD: True} | values))
    params.values.update({"FM_LaneKeepMode": 0, "IsRhdDetected": False})
    lat = FmLateral(SimpleNamespace(brand='rivian'), params)
    lat.lane_guard.update = lambda *a, **k: 0.0
    return lat

  def test_turn_direction_from_controlsds_submaster_without_polling(self, monkeypatch):
    """The turn desire is read from the SubMaster controlsd passes in; FmLateral never opens or polls one (msgq's poll
    sleeps when no message is waiting, 4 of every 5 steps for a 20 Hz service)."""
    from openpilot.cereal import messaging

    def no_submaster(*args, **kwargs):
      raise AssertionError("FmLateral must not open a SubMaster of its own")
    monkeypatch.setattr(messaging, "SubMaster", no_submaster)

    lat = self._lateral()
    CS = SimpleNamespace(vEgo=10 * CV.MPH_TO_MS, steeringPressed=False, leftBlinker=False, rightBlinker=True)
    for _ in range(FADE_STEPS):
      lat.update(0.09, SimpleNamespace(), CS, True, False, 0.001, self.ControlsdSM(RIGHT))
    assert lat.update(0.03, SimpleNamespace(), CS, True, False, 0.001, self.ControlsdSM(RIGHT)) == pytest.approx(0.081, rel=0.01)
    # no turn desire seen yet (modeld hasn't published), or none passed at all: the request is untouched
    for sm in (self.ControlsdSM(RIGHT, seen=False), None):
      lat = self._lateral()
      for _ in range(FADE_STEPS):
        lat.update(0.09, SimpleNamespace(), CS, True, False, 0.001, sm)
      assert lat.update(0.03, SimpleNamespace(), CS, True, False, 0.001, sm) == 0.03

  def test_turn_hold_and_lead_off_skips_the_update(self):
    lat = self._lateral(**{PARAM_HOLD: False, PARAM_LEAD_ADAPTIVE: True, PARAM_LEAD_GATE: True, PARAM_LEAD_YIELD: True})
    lat.turn_assist.update = lambda *a, **k: (_ for _ in ()).throw(AssertionError("not with Turn Hold and Lead off"))
    CS = SimpleNamespace(vEgo=4.0, steeringPressed=False, leftBlinker=False, rightBlinker=True)
    assert lat.update(0.05, SimpleNamespace(), CS, True, False, 0.0, self.ControlsdSM(RIGHT)) == 0.05

  def test_turn_lead_alone_runs(self):
    lat = self._lateral(**{PARAM_HOLD: False, PARAM_LEAD: True})
    CS = SimpleNamespace(vEgo=10 * CV.MPH_TO_MS, steeringPressed=False, leftBlinker=False, rightBlinker=True)
    out = 0.0
    for _ in range(FADE_STEPS):
      out = lat.update(0.0, path(15.0), CS, True, False, 0.0, self.ControlsdSM(RIGHT))
    assert out > 0.02

  def test_controlsd_subscribes_to_the_turn_desire_and_passes_its_submaster(self):
    import ast
    import openpilot.selfdrive.controls as controls
    tree = ast.parse((Path(controls.__file__).parent / "controlsd.py").read_text())
    submasters = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and ast.unparse(n.func) == "messaging.SubMaster"]
    assert len(submasters) == 1 and "'modelDataV2SP'" in ast.unparse(submasters[0])
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and ast.unparse(n.func) == "self.fm_lateral.update"]
    assert len(calls) == 1 and [ast.unparse(x) for x in calls[0].args[-2:]] == ["self.curvature", "self.sm"]


def test_controlsd_ext_logs_turn_assist_in_fm_controls():
  from openpilot.cereal import messaging
  from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
  a = hold()
  run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
  run(a, 0.03, RIGHT, actual=0.001)
  cs = messaging.new_message('carState').carState
  cs.vEgo = 3.0
  host = SimpleNamespace(LaC=SimpleNamespace(), fm_lateral=SimpleNamespace(turn_assist=a))
  ta = ControlsExt.fm_controls_ext(host, {'carState': cs}).fmControls.turnAssist
  assert ta.sign == 1 and ta.holdActive and ta.holdFloor > 0.0
  host = SimpleNamespace(LaC=SimpleNamespace())  # no FmLateral (a test host): nothing logged
  assert ControlsExt.fm_controls_ext(host, {'carState': cs}).fmControls.turnAssist.sign == 0


class TestSettings:
  STEERING_YAML = Path(fm_turn_assist.__file__).parents[3] / "sunnylink" / "settings_ui_src" / "pages" / "steering.yaml"

  def _sections(self):
    return {s["id"]: s for s in yaml.safe_load(self.STEERING_YAML.read_text())["sections"]}

  def test_defaults_off(self):
    from openpilot.common.params import Params
    params = Params()
    for key in SETTING_KEYS + LEAD_TOGGLES:
      assert params.get_default_value(key) is False, key
    assert params.get_default_value(PARAM_MIN_RADIUS) == pytest.approx(MIN_RADIUS_DEFAULT)
    assert params.get_default_value(PARAM_LEAD_MIN_SPEED) == pytest.approx(7.0)
    assert params.get_default_value(PARAM_LEAD_GATE_DIST) == pytest.approx(5.0)
    for key in FOLDED_KEYS:  # still declared, so switching back to fm-dev-nav-settle keeps them; no default written
      assert params.get_default_value(key) is None, key

  def test_fresh_install_is_off(self):
    a = FmTurnAssist(FakeParams())
    assert not a.enabled and not a.hold_path and not a.hold_driver and not a.hold_follow_exit
    assert not a.restart_after_stop and not a.phase_in and not a.carry_enabled
    assert not a.lead_enabled and not a.lead_adaptive and not a.lead_gate and not a.lead_yield
    assert a.lead_min_speed == pytest.approx(7.0 * CV.MPH_TO_MS) and a.lead_gate_dist == 5.0
    assert a.max_curvature == pytest.approx(1 / MIN_RADIUS_DEFAULT)

  def test_fresh_install_with_real_params_is_off(self):
    """Unset keys read through Params' declared defaults: Turn Lead and its options off, 7 mph, 5 m, 12 m."""
    from openpilot.common.params import Params
    from openpilot.common.prefix import OpenpilotPrefix
    with OpenpilotPrefix():  # an empty params store, as on a fresh install
      a = FmTurnAssist(Params())
    assert not a.lead_enabled and not a.lead_adaptive and not a.lead_gate and not a.lead_yield and not a.carry_enabled
    assert a.lead_min_speed == pytest.approx(7.0 * CV.MPH_TO_MS) and a.lead_gate_dist == pytest.approx(5.0)
    assert a.max_curvature == pytest.approx(1 / MIN_RADIUS_DEFAULT)

  def test_smooth_turn_start_needs_turn_hold_or_lead(self):
    (item,) = [i for i in self._sections()["fm_smoothing"]["items"] if i["key"] == PARAM_PHASE_IN]
    (any_of,) = item["enablement"]
    assert any_of["type"] == "any"
    assert {c["key"] for c in any_of["conditions"]} == {PARAM_HOLD, PARAM_LEAD}
    assert "Turn Lead" in item["description"] and "Turn Hold" in item["description"]

  def test_smooth_turn_start_goes_first_in_fm_smoothing(self):
    keys = [item["key"] for item in self._sections()["fm_smoothing"]["items"]]
    assert keys == [PARAM_PHASE_IN, "FM_LowSpeedCorrectionStrength", "FM_LowSpeedCurvatureFloor", "FM_StraightPullAwayFloor"]

  def test_turn_hold_settings_sub_page(self):
    turns = self._sections()["fm_turns"]
    assert [item["key"] for item in turns["items"]] == [PARAM_HOLD, PARAM_LEAD, PARAM_CARRY]
    sub = turns["sub_panels"][0]
    assert sub["label"] == "Turn Hold Settings" and sub["trigger_key"] == PARAM_HOLD
    assert [item["key"] for item in sub["items"]] == [PARAM_HOLD_PATH, PARAM_HOLD_DRIVER, PARAM_HOLD_FOLLOW_EXIT,
                                                      PARAM_RESTART_AFTER_STOP]
    assert all(item["widget"] == "toggle" for item in sub["items"])

  def test_turn_lead_settings_sub_page(self):
    turns = self._sections()["fm_turns"]
    assert [s["id"] for s in turns["sub_panels"]] == ["turn_hold_settings", "turn_lead_settings", "pre_post_turn"]
    sub = turns["sub_panels"][1]
    assert sub["label"] == "Turn Lead Settings" and sub["trigger_key"] == PARAM_LEAD
    assert sub["trigger_condition"] == {"type": "param", "key": PARAM_LEAD, "equals": True}
    items = {item["key"]: item for item in sub["items"]}
    assert list(items) == [PARAM_LEAD_MIN_SPEED, PARAM_LEAD_ADAPTIVE, PARAM_LEAD_GATE, PARAM_LEAD_GATE_DIST, PARAM_LEAD_YIELD,
                           PARAM_MIN_RADIUS]
    assert [items[k]["title"] for k in items] == ["Turn Lead Min Speed", "Adaptive Look-Ahead", "Wait for the Corner",
                                                  "Turn Lead Start Distance", "Turn Lead Yields to Your Hands",
                                                  "Turn Path Tightest Radius"]
    for key, lo, hi in ((PARAM_LEAD_MIN_SPEED, 2, 7), (PARAM_LEAD_GATE_DIST, 3, 10), (PARAM_MIN_RADIUS, 8, 20)):
      assert items[key]["widget"] == "option" and (items[key]["min"], items[key]["max"], items[key]["step"]) == (lo, hi, 1)
    for key in (PARAM_LEAD_ADAPTIVE, PARAM_LEAD_GATE, PARAM_LEAD_YIELD):
      assert items[key]["widget"] == "toggle"
      assert "On by default" not in items[key]["description"]
    assert {c["key"] for c in items[PARAM_LEAD_GATE_DIST]["enablement"]} == {PARAM_LEAD, PARAM_LEAD_GATE}
    (any_of,) = items[PARAM_MIN_RADIUS]["enablement"]
    assert {c["key"] for c in any_of["conditions"]} == {PARAM_LEAD, PARAM_HOLD_PATH}
    assert "Hold Path Source" in items[PARAM_MIN_RADIUS]["description"]
    # the radius has exactly one entry, here
    assert self.STEERING_YAML.read_text().count(f"key: {PARAM_MIN_RADIUS}") == 1

  def test_every_turn_assist_setting_is_declared(self):
    from openpilot.common.params import Params
    params = Params()
    for key in SETTING_KEYS + LEAD_TOGGLES + (PARAM_LEAD_MIN_SPEED, PARAM_LEAD_GATE_DIST, PARAM_MIN_RADIUS):
      assert params.check_key(key), key

  def test_finish_turn_after_signal_cancels_on_the_turns_section(self):
    """A toggle on the Turns section itself (not a sub-page), enabled by Turn Hold or Turn Lead; Carry Only at Turn-In is
    folded into its description, with no entry of its own."""
    turns = self._sections()["fm_turns"]
    (item,) = [i for i in turns["items"] if i["key"] == PARAM_CARRY]
    assert item["widget"] == "toggle" and item["title"] == "Finish Turn After Signal Cancels"
    (any_of,) = item["enablement"]
    assert any_of["type"] == "any" and {c["key"] for c in any_of["conditions"]} == {PARAM_HOLD, PARAM_LEAD}
    assert all(c["equals"] is True for c in any_of["conditions"])
    assert "Boost" not in item["description"] and "On by default" not in item["description"]
    assert "45 degrees" in item["description"]
    assert all(PARAM_CARRY not in [i["key"] for i in sub["items"]] for sub in turns["sub_panels"])
    assert self.STEERING_YAML.read_text().count(f"key: {PARAM_CARRY}\n") == 1

  def test_folded_keys_not_in_the_settings(self):
    text = self.STEERING_YAML.read_text()
    for key in FOLDED_KEYS:
      assert key not in text
    assert "Nov 20" not in text


def lead(**values):
  return FmTurnAssist(FakeParams(**({PARAM_LEAD: True} | values)))


class TestTurnLead:
  """Turn Lead (FM_TurnLeadEnabled): starts the turn on time from the planned path's curve ahead. From StarPilot's Model
  Turn Assist "turn lead"."""

  def test_off_by_default(self):
    a = FmTurnAssist(FakeParams())
    assert run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(10.0), actual=0.0) == 0.0

  def test_path_geometry_sign(self):
    p = path(10.0, start=0.0)
    assert plan_circle_curvature(p.position.x, p.position.y, 7.0) == pytest.approx(0.1, rel=0.05)  # right is positive
    q = path(10.0, start=0.0, side=-1)
    assert plan_circle_curvature(q.position.x, q.position.y, 7.0) == pytest.approx(-0.1, rel=0.05)

  def test_turns_in_when_model_is_late(self):
    a = lead()
    assert a.enabled and not a.hold_enabled
    out = run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    assert 0.02 < out <= 1 / MIN_RADIUS_DEFAULT + 1e-9

  def test_left_turn(self):
    assert run(lead(), 0.0, LEFT, steps=FADE_STEPS, model=path(15.0, side=-1), actual=0.0) < -0.02

  def test_never_toward_the_other_side(self):
    assert run(lead(), 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0, side=-1), actual=0.0) == 0.0

  @pytest.mark.parametrize("setting,radius", [(MIN_RADIUS_DEFAULT, MIN_RADIUS_DEFAULT), (8.0, 8.0), (3.0, 8.0), (40.0, 20.0),
                                              (None, MIN_RADIUS_DEFAULT), ("x", MIN_RADIUS_DEFAULT),
                                              (float("nan"), MIN_RADIUS_DEFAULT)])
  def test_tight_radius_cap(self, setting, radius):
    """Turn Path Tightest Radius caps Turn Lead's request (a 5 m corner asks for more than any setting allows)."""
    a = lead(**{PARAM_MIN_RADIUS: setting})
    out = run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(5.0, start=0.0), actual=0.0)
    assert out == pytest.approx(1 / radius)

  def test_hands_back_as_truck_catches_up(self):
    a = lead()
    half = run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    assert run(a, 0.0, RIGHT, model=path(15.0), actual=half) == 0.0  # turning as tightly as the path asks: model again

  def test_never_reduces_model_request(self):
    assert run(lead(), 0.09, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0) == 0.09

  @pytest.mark.parametrize("mph,active", [(5, False), (9, True), (13, True), (17, False)])
  def test_speed_band(self, mph, active):
    out = run(lead(), 0.0, RIGHT, mph=mph, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    assert (out > 0.0) == active

  @pytest.mark.parametrize("setting,mph,active", [(7.0, 6, False), (3.0, 2, False), (3.0, 4, True), (2.0, 3, True),
                                                  (1.0, 1.5, False), (10.0, 6, False), (None, 6, False), ("x", 8, True)])
  def test_min_speed_setting(self, setting, mph, active):
    """Below and above Turn Lead Min Speed (clipped to 2..7 mph; unset or unreadable -> 7)."""
    out = run(lead(**{PARAM_LEAD_MIN_SPEED: setting}), 0.0, RIGHT, mph=mph, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    assert (out > 0.0) == active

  def test_full_strength_about_2_mph_above_min(self):
    full = run(lead(**{PARAM_LEAD_MIN_SPEED: 3.0}), 0.0, RIGHT, mph=9, steps=FADE_STEPS, model=path(15.0, start=0.0),
               actual=0.0)  # a circle: same curve at any distance
    b = lead(**{PARAM_LEAD_MIN_SPEED: 3.0})
    assert run(b, 0.0, RIGHT, mph=5.3, steps=FADE_STEPS, model=path(15.0, start=0.0), actual=0.0) == pytest.approx(full, rel=0.05)

  def test_not_when_stopping_short(self):
    assert run(lead(), 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0, a_ego=-3.0) == 0.0

  def test_not_when_model_steers_away(self):
    assert run(lead(), -0.01, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0) == -0.01

  def test_not_during_lane_change(self):
    assert run(lead(), 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0, lane_changing=True) == 0.0

  def test_not_while_openpilot_isnt_steering(self):
    assert run(lead(), 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0, lat=False) == 0.0

  def test_no_turn_no_lead(self):
    assert run(lead(), 0.0, NONE, steps=FADE_STEPS, model=path(15.0), actual=0.0) == 0.0

  def test_no_path_no_lead(self):
    a = lead()
    assert run(a, 0.0, RIGHT, steps=FADE_STEPS, model=None, actual=0.0) == 0.0
    assert run(a, 0.0, RIGHT, steps=FADE_STEPS, model=SimpleNamespace(), actual=0.0) == 0.0

  def test_hold_keeps_what_lead_asked(self):
    """Turn Lead runs before Turn Hold: the hold's peak is Turn Lead's request, and the floor holds it."""
    a = lead(**{PARAM_HOLD: True})
    asked = run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    assert a.peak == pytest.approx(asked)
    assert run(a, 0.0, RIGHT, model=path(None), actual=0.0) == pytest.approx(asked * 0.9, rel=0.02)

  def test_nudge_needs_turn_hold(self):
    """Turn Nudge is part of Turn Hold: Turn Lead alone never nudges."""
    a = lead(**{PARAM_LEAD_MIN_SPEED: 7.0})  # below 7 mph Turn Lead adds nothing, so only a nudge could
    model = path(12, start=0.0)
    run(a, 0.0, RIGHT, mph=4, steps=FADE_STEPS, actual=0.0, model=model)
    assert run(a, 0.0, RIGHT, mph=4, steps=30, actual=0.0, model=model, pressed=True, torque=-1.0) == 0.0


class TestAdaptiveLookAhead:
  """Adaptive Look-Ahead: Turn Lead reads 0.5 s (gentle) to 0.9 s (tight) ahead instead of a fixed 1.3 s, at least 3 m."""

  def _assist(self, adaptive):
    return lead(**{PARAM_LEAD_MIN_SPEED: 2.0, PARAM_LEAD_ADAPTIVE: adaptive})

  def test_off_is_the_fixed_1_3_s(self):
    v = 13 * CV.MPH_TO_MS
    assert self._assist(False).lead_distance(path(12, start=2.0), 1, v) == pytest.approx(1.3 * v)

  @pytest.mark.parametrize("radius,seconds", [(60, 0.5), (10, 0.9)])
  def test_on_reads_by_how_tight(self, radius, seconds):
    v = 14 * CV.MPH_TO_MS
    assert self._assist(True).lead_distance(path(radius, start=0.0), 1, v) == pytest.approx(max(seconds * v, 3.0), rel=0.15)

  @pytest.mark.parametrize("radius", [8, 12, 20, 50])
  def test_plan_max_curvature_finds_the_corner(self, radius):
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import plan_max_curvature
    for side in (1, -1):
      assert plan_max_curvature(path(radius, start=6.0, side=side), side, 17.0) == pytest.approx(1 / radius, rel=0.2)
      assert plan_max_curvature(path(radius, start=6.0, side=side), -side, 17.0) == 0.0
    assert plan_max_curvature(path(None), 1, 17.0) == 0.0
    assert plan_max_curvature(path(12, start=20.0), 1, 17.0) == 0.0  # past the window
    assert plan_max_curvature(SimpleNamespace(), 1, 17.0) == 0.0

  def test_on_never_closer_than_3_m(self):
    assert self._assist(True).lead_distance(path(None), 1, 3 * CV.MPH_TO_MS) == pytest.approx(3.0)

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
    assert side * run(self._assist(True), 0.0, direction, mph=13, steps=FADE_STEPS, actual=0.0, model=model) > 0.03


class TestWaitForTheCorner:
  """Wait for the Corner + Turn Lead Start Distance: no Turn Lead until the path has turned 10 degrees toward the turn
  within the set distance."""

  def _lead(self, model, direction, gate=None, mph=13):
    values = {PARAM_LEAD_MIN_SPEED: 2.0}
    if gate is not None:
      values |= {PARAM_LEAD_GATE: True, PARAM_LEAD_GATE_DIST: gate}
    return run(lead(**values), 0.0, direction, mph=mph, steps=FADE_STEPS, actual=0.0, model=model)

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

  def test_distance_off_without_the_toggle(self):
    """Turn Lead Start Distance does nothing while Wait for the Corner is off."""
    model = path(12, start=5.5)
    a = lead(**{PARAM_LEAD_MIN_SPEED: 2.0, PARAM_LEAD_GATE_DIST: 3.0})
    assert run(a, 0.0, RIGHT, mph=13, steps=FADE_STEPS, actual=0.0, model=model) == pytest.approx(self._lead(model, RIGHT))

  def test_bend_the_other_way_does_not_open_it(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import plan_turn_onset_toward
    assert math.isinf(plan_turn_onset_toward(path(12, start=1.0, side=-1), 1))
    assert 3.0 < plan_turn_onset_toward(path(12, start=1.0, side=1), 1) < 4.5  # 2-3 m into the corner
    assert math.isinf(plan_turn_onset_toward(path(None), 1))
    assert math.isinf(plan_turn_onset_toward(SimpleNamespace(), 1))

  @pytest.mark.parametrize("value,dist", [(50.0, 10.0), (1.0, 3.0), ("x", 5.0), (None, 5.0), (float("nan"), 5.0), (7.0, 7.0)])
  def test_gate_setting_is_clamped(self, value, dist):
    assert FmTurnAssist(FakeParams(**{PARAM_LEAD_GATE_DIST: value})).lead_gate_dist == dist


class TestLeadYield:
  """Turn Lead Yields to Your Hands (FM_TurnLeadYield, off by default): a press against the turn while Turn Lead is
  adding pauses it before 10 deg turned (back 0.5 s after you let go) and ends it for the turn after that
  (000000af--8 +52.3 s: he pushed out at 1.75 Nm and Turn Lead kept adding)."""

  def _assist(self, on, **extra):
    return lead(**({PARAM_LEAD_YIELD: on} | extra))

  def _leading(self, a):
    out = run(a, 0.0, RIGHT, steps=FADE_STEPS, model=path(15.0), actual=0.0)
    assert out > 0.02
    return out

  def _press(self, a, steps, torque=1.0):  # positive (left) torque is against a right turn
    return run(a, 0.0, RIGHT, steps=steps, model=path(15.0), actual=0.0, pressed=True, torque=torque)

  def test_off_keeps_leading_against_your_hands(self):
    a = self._assist(False)
    lead_out = self._leading(a)
    assert self._press(a, 60) == pytest.approx(lead_out)

  def test_pauses_before_turn_in_and_comes_back(self):
    a = self._assist(True)
    lead_out = self._leading(a)
    assert self._press(a, 30) == 0.0
    m = turn_assist_msg()
    a.fill_log(m)
    assert m.leadYield == 1
    assert run(a, 0.0, RIGHT, steps=30, model=path(15.0), actual=0.0) == 0.0  # 0.3 s after letting go: still paused
    assert run(a, 0.0, RIGHT, steps=30, model=path(15.0), actual=0.0) == pytest.approx(lead_out)  # back after 0.5 s

  def test_ends_for_the_turn_once_turning(self):
    a = self._assist(True)
    self._leading(a)
    a.turned = math.radians(15.0)
    assert self._press(a, 30) == 0.0
    assert run(a, 0.0, RIGHT, steps=300, model=path(15.0), actual=0.0) == 0.0
    m = turn_assist_msg()
    a.fill_log(m)
    assert m.leadYield == 2

  def test_firm_press_acts_at_once(self):
    a = self._assist(True)
    self._leading(a)
    assert self._press(a, 1, torque=1.6) == 0.0

  def test_brief_light_press_ignored(self):
    a = self._assist(True)
    lead_out = self._leading(a)
    assert self._press(a, 10, torque=0.8) == pytest.approx(lead_out)

  def test_press_before_lead_starts_does_nothing(self):
    a = self._assist(True)
    run(a, 0.0, RIGHT, steps=60, model=path(None), actual=0.0, pressed=True, torque=1.0)  # straight path: no lead yet
    assert self._leading(a) > 0.02

  def test_press_into_the_turn_never_yields(self):
    a = self._assist(True)
    lead_out = self._leading(a)
    assert self._press(a, 60, torque=-1.0) == pytest.approx(lead_out)

  def test_new_turn_resets(self):
    a = self._assist(True)
    self._leading(a)
    a.turned = math.radians(15.0)
    self._press(a, 30)
    run(a, 0.0, NONE, steps=5)
    assert self._leading(a) > 0.02

  def test_hold_still_holds_after_lead_yields(self):
    """With Turn Hold on too, a yield only stops Turn Lead: the hold's own rules decide whether it lets go."""
    a = self._assist(True, **{PARAM_HOLD: True})
    asked = self._leading(a)
    assert a.peak == pytest.approx(asked)
    a.turned = math.radians(15.0)
    out = self._press(a, 1, torque=1.6)  # firm: Turn Lead off, and the hold released (a 1.5 Nm press past 10 deg)
    assert a.lead_off and a.hold_released and out == 0.0


class TestTurnHoldOnlyUnchanged:
  """With Turn Lead off, its options (stored on, as from fm-dev-nav-settle) change nothing: Turn Hold, Smooth Turn Start
  and the log behave as before plan PR 11 (tools/fm/tests/pr11_turn_lead_equivalence.py also compares against PR 10's
  module itself)."""
  LEAD_OPTIONS_ON = {PARAM_LEAD_ADAPTIVE: True, PARAM_LEAD_GATE: True, PARAM_LEAD_GATE_DIST: 3.0, PARAM_LEAD_YIELD: True,
                     PARAM_LEAD_MIN_SPEED: 2.0}

  @pytest.mark.parametrize("seed", range(4))
  def test_lead_options_without_turn_lead_change_nothing(self, seed):
    rng = np.random.default_rng(seed)
    opts = {PARAM_HOLD_DRIVER: bool(seed & 1), PARAM_HOLD_PATH: True, PARAM_PHASE_IN: bool(seed & 2),
            PARAM_HOLD_FOLLOW_EXIT: True, PARAM_RESTART_AFTER_STOP: True}
    a, b = hold(**opts), hold(**(opts | self.LEAD_OPTIONS_ON))
    models = [path(r, start=st, side=sd) for r in (8.0, 12.0, 20.0) for st in (0.0, 4.0) for sd in (1, -1)] + [None]
    model, direction = models[0], RIGHT
    for i in range(4000):
      if i % 300 == 0:
        model, direction = models[int(rng.integers(len(models)))], (RIGHT, LEFT, NONE)[int(rng.integers(3))]
      k = 0.08 * math.sin(i / 60) + float(rng.normal(0, 0.005))
      args = {"mph": float(rng.uniform(0, 18)), "actual": k * 0.7, "pressed": bool(rng.random() < 0.1),
              "torque": float(rng.uniform(-2, 2)), "model": model, "lat": bool(rng.random() < 0.97)}
      assert run(a, k, direction, **args) == run(b, k, direction, **args)
      ma, mb = turn_assist_msg(), turn_assist_msg()
      a.fill_log(ma)
      b.fill_log(mb)
      assert ma.to_dict() == mb.to_dict()
    assert not b.lead_off and not b.lead_paused and b.lead_prev == 0.0


def carry(**values):
  return FmTurnAssist(FakeParams(**({PARAM_HOLD: True, PARAM_CARRY: True} | values)))


class TestCarryThrough:
  """Finish Turn After Signal Cancels: the turn goes on for Turn Assist while the truck is still turning that way.
  Ported from ca08aac9; there the class ran with Carry Only at Turn-In off, here it is folded on, which these turns (the
  blinker off near turn-in, the truck still turning tighter) don't reach."""

  @pytest.mark.parametrize("direction, sign", [(RIGHT, 1), (LEFT, -1)])
  def test_hold_carries_while_turning(self, direction, sign):
    a = carry()
    run(a, sign * 0.09, direction, steps=FADE_STEPS, actual=sign * 0.001)
    out = run(a, sign * 0.01, NONE, steps=50, actual=sign * 0.08)  # blinker off, truck still turning
    assert a.carry_sign == sign
    assert sign * out > 0.06

  def test_off_fades_as_before(self):
    a = carry(**{PARAM_CARRY: False})
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    assert run(a, 0.01, NONE, steps=FADE_STEPS, actual=0.08) == 0.01

  def test_not_turning_does_not_carry(self):
    a = carry()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.01, NONE, actual=0.005)
    assert a.carry_sign == 0
    run(a, 0.01, NONE, actual=0.08)  # turning later doesn't start it: needs a new turn desire
    assert a.carry_sign == 0

  def test_ends_when_straight_then_fades(self):
    a = carry()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.01, NONE, steps=20, actual=0.08)
    first = run(a, 0.01, NONE, actual=0.005)
    assert a.carry_sign == 0
    assert 0.01 < first  # the usual cancel fade
    assert run(a, 0.01, NONE, steps=FADE_STEPS, actual=0.005) == 0.01
    run(a, 0.01, NONE, actual=0.08)
    assert a.carry_sign == 0  # never restarts without a new turn desire

  @pytest.mark.parametrize("opts", [{"lat": False}, {"lane_changing": True}, {"mph": 21},
                                    {"pressed": True, "torque": HOLD_AGAINST_TORQUE + 0.1}])
  def test_ends_at_once(self, opts):
    a = carry()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.01, NONE, steps=5, actual=0.08)
    run(a, 0.01, NONE, actual=0.08, **opts)
    assert a.carry_sign == 0

  @pytest.mark.parametrize("direction, sign", [(RIGHT, 1), (LEFT, -1)])
  def test_firm_press_against_ends_it_at_once(self, direction, sign):
    """HOLD_AGAINST_TORQUE (0.5 Nm) against the carried turn ends it the same frame; into the turn it doesn't."""
    assert HOLD_AGAINST_TORQUE == 0.5
    a = carry()
    run(a, sign * 0.09, direction, steps=FADE_STEPS, actual=sign * 0.001)
    run(a, sign * 0.01, NONE, steps=5, actual=sign * 0.08)
    run(a, sign * 0.01, NONE, actual=sign * 0.08, pressed=True, torque=-sign * 1.5)  # into the turn
    assert a.carry_sign == sign
    run(a, sign * 0.01, NONE, actual=sign * 0.08, pressed=True, torque=sign * 0.49)  # under 0.5 Nm: not yet
    assert a.carry_sign == sign
    run(a, sign * 0.01, NONE, actual=sign * 0.08, pressed=True, torque=sign * 0.5)
    assert a.carry_sign == 0

  def test_brief_blip_against_does_not_end_it(self):
    a = carry()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.01, NONE, steps=5, actual=0.08)
    run(a, 0.01, NONE, pressed=True, torque=0.14, actual=0.08)
    assert a.carry_sign == 1
    run(a, 0.01, NONE, steps=int(HOLD_AGAINST_TIME / DT_CTRL) + 1, pressed=True, torque=0.14, actual=0.08)
    assert a.carry_sign == 0

  def test_max_time(self):
    a = carry()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.01, NONE, steps=int(CARRY_MAX_TIME / DT_CTRL) + 2, actual=0.08, mph=3)
    assert a.carry_sign == 0

  def test_lead_keeps_working(self):
    """Turn Lead follows the carried turn's sign (Turn Lead alone, no Turn Hold)."""
    a = FmTurnAssist(FakeParams(**{PARAM_LEAD: True, PARAM_LEAD_MIN_SPEED: 2.0, PARAM_CARRY: True}))
    model = path(10, start=0.0)
    run(a, 0.0, RIGHT, mph=5, steps=FADE_STEPS, actual=0.012, model=model)
    assert run(a, 0.0, NONE, mph=5, steps=10, actual=0.015, model=model) > 0.05
    assert a.carry_sign == 1 and a.log_source == fm_turn_assist.SOURCE_CARRY and a.log_lead > 0.0

  def test_lead_off_without_carry(self):
    a = FmTurnAssist(FakeParams(**{PARAM_LEAD: True, PARAM_LEAD_MIN_SPEED: 2.0}))
    model = path(10, start=0.0)
    run(a, 0.0, RIGHT, mph=5, steps=FADE_STEPS, actual=0.012, model=model)
    assert run(a, 0.0, NONE, mph=5, steps=10, actual=0.015, model=model) == 0.0

  def test_other_side_turn_is_a_new_turn(self):
    a = carry()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.01, NONE, steps=5, actual=0.08)
    run(a, -0.01, LEFT, actual=0.08)
    assert a.carry_sign == 0 and a.sign == -1

  def test_inert_with_turn_hold_and_lead_off(self):
    a = FmTurnAssist(FakeParams(**{PARAM_CARRY: True}))
    assert not a.enabled and a.carry_enabled
    for k, d, act in [(0.09, RIGHT, 0.001), (0.01, NONE, 0.08)] * 60:
      assert run(a, k, d, steps=5, actual=act) == k
      assert a.carry_sign == 0 and a.log_source == fm_turn_assist.SOURCE_NONE

  def test_switching_turn_hold_off_mid_carry_ends_it(self):
    params = FakeParams(**{PARAM_HOLD: True, PARAM_CARRY: True})
    a = FmTurnAssist(params)
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.01, NONE, steps=5, actual=0.08)
    assert a.carry_sign == 1
    params.values[PARAM_HOLD] = False
    a.read_params()
    assert run(a, 0.01, NONE, actual=0.08) == 0.01 and a.carry_sign == 0


class TestCarryTurnInOnly:
  """Carry Only at Turn-In (folded, always on): a blinker cancel late in the turn, or once the truck is straightening out,
  ends the turn. FM_TurnCarryTurnInOnly is not read."""

  def test_cancel_at_turn_in_still_carries(self):
    a = carry()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.02)  # ~6 deg turned
    run(a, 0.01, NONE, actual=0.08)
    assert a.carry_sign == 1

  @pytest.mark.parametrize("old", [None, True, False])
  def test_cancel_late_in_turn_fades_instead(self, old):
    """With the old key unset, on, or stored off (ca08aac9 then carried): the usual cancel fade."""
    a = carry(**({} if old is None else {CARRY_TURN_IN_KEY: old}))
    run(a, 0.09, RIGHT, steps=150, actual=0.1)  # 0.1 1/m at 10 mph: about 38 deg
    run(a, 0.03, RIGHT, steps=50, actual=0.1)  # the model eases, Turn Hold's floor holds it: about 51 deg
    assert 45 < math.degrees(a.turned) < 75
    first = run(a, 0.01, NONE, actual=0.1)
    assert a.carry_sign == 0
    assert 0.01 < first < 0.09  # the usual cancel fade, not a carry
    assert run(a, 0.01, NONE, steps=FADE_STEPS, actual=0.1) == 0.01
    assert a.carry_sign == 0

  @pytest.mark.parametrize("old", [None, False])
  def test_cancel_while_straightening_fades_instead(self, old):
    a = carry(**({} if old is None else {CARRY_TURN_IN_KEY: old}))
    run(a, 0.09, RIGHT, steps=50, actual=0.1)  # tightest 0.1, ~13 deg turned
    run(a, 0.05, RIGHT, steps=5, actual=0.05)  # unwinding to half
    run(a, 0.01, NONE, actual=0.05)
    assert a.carry_sign == 0

  def test_old_off_does_not_carry_late(self):
    """ca08aac9's test_off_carries_as_before: with the old key off it carried 51 deg in; folded, it doesn't."""
    a = carry(**{CARRY_TURN_IN_KEY: False})
    run(a, 0.09, RIGHT, steps=200, actual=0.1)
    run(a, 0.01, NONE, actual=0.05)
    assert a.carry_sign == 0

  def test_just_under_the_limits_carries(self):
    a = carry()
    run(a, 0.09, RIGHT, steps=100, actual=0.1)  # ~26 deg
    run(a, 0.075, RIGHT, steps=2, actual=0.075)
    run(a, 0.01, NONE, actual=0.075)  # 75% of the tightest: still turning in
    assert a.carry_sign == 1

  def test_00000073_8_left_exit(self):
    """--8 +35.2 s: blinker self-cancelled 51 deg into a left at 10 mph, truck at 16 m after 9 m at the tightest."""
    a = carry()
    run(a, -0.11, LEFT, mph=10, steps=200, actual=-0.11)  # ~56 deg
    run(a, -0.04, LEFT, mph=10, steps=20, actual=-0.0625)
    run(a, -0.016, NONE, mph=10, actual=-0.0625)
    assert a.carry_sign == 0

  def test_old_key_never_read(self):
    params = FakeParams(**{PARAM_HOLD: True, PARAM_LEAD: True, PARAM_CARRY: True, CARRY_TURN_IN_KEY: False})
    a = FmTurnAssist(params)
    for i in range(300):
      run(a, 0.09, RIGHT if i % 50 < 30 else NONE, actual=0.08, model=path(10.0, start=0.0))
      if i % 100 == 0:
        a.read_params()
    assert PARAM_CARRY in params.reads and CARRY_TURN_IN_KEY not in params.reads


class TestCarryLog:
  @staticmethod
  def logged(a):
    m = turn_assist_msg()
    a.fill_log(m)
    return m

  def test_carry_source_and_exit_fade(self):
    a = carry()
    run(a, 0.09, RIGHT, steps=FADE_STEPS, actual=0.001)
    run(a, 0.01, NONE, actual=0.08)
    m = self.logged(a)
    assert str(m.source) == "carry" and m.sign == 1 and m.holdFloor > 0.0
    run(a, 0.01, NONE, actual=0.005)  # carry ends: the usual cancel fade
    m = self.logged(a)
    assert str(m.source) == "none" and m.sign == 1 and m.exitFade > 0.0

  def test_logging_changes_nothing(self):
    opts = {PARAM_HOLD: True, PARAM_LEAD: True, PARAM_LEAD_MIN_SPEED: 2.0, PARAM_HOLD_DRIVER: True, PARAM_CARRY: True}
    a, b = FmTurnAssist(FakeParams(**opts)), FmTurnAssist(FakeParams(**opts))
    model = path(10, start=0.0)
    for k, act, d in [(0.0, 0.0, RIGHT), (0.02, 0.05, RIGHT), (0.01, 0.09, RIGHT), (0.01, 0.05, NONE)] * 30:
      x = run(a, k, d, mph=5, actual=act, model=model)
      a.fill_log(turn_assist_msg())
      assert x == run(b, k, d, mph=5, actual=act, model=model)
