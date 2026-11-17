"""FM Pilot: FM_StraightPullAwayFloor (Calmer Straight Pull-Away): the decision, and what it does to the request."""
import math

import pytest

from openpilot.sunnypilot.modeld_v2.fm_pull_away import BLEND_S, DT, PRESS_HOLD_S, STRAIGHT_FLOOR, StraightPullAway

USUAL = 1.0


def step(p, v, enabled=True, base=USUAL, blinker=False, desire=False, angle=0.0, pressed=False):
  return p.update(enabled, base, v, blinker, desire, angle, pressed)


def curvature(lat_accel, v_ego, floor):
  return lat_accel / max(floor, v_ego) ** 2  # as modeld_v2's get_action_from_model


def stopped(**kw):
  p = StraightPullAway()
  step(p, 0.0, **kw)
  return p


def test_off_is_the_usual_floor_always():
  p = StraightPullAway()
  for v in (0.0, 0.5, 1.0, 2.0, 5.0):
    assert step(p, v, enabled=False) == USUAL


def test_straight_pull_away_from_a_stop_uses_the_calmer_floor():
  p = stopped()
  for v in (0.31, 0.7, 1.0, 1.8, 2.4):
    assert step(p, v) == pytest.approx(STRAIGHT_FLOOR)
  assert step(p, 2.6) == USUAL  # both floors agree from here
  assert curvature(0.5, 2.6, STRAIGHT_FLOOR) == curvature(0.5, 2.6, USUAL)


def test_slowing_into_crawl_speed_without_a_stop_is_unchanged():
  p = StraightPullAway()
  for v in (6.0, 3.0, 2.4, 1.5, 0.8, 0.4):  # a rolling crawl turn (garage ramp) keeps its full request
    assert step(p, v) == USUAL


@pytest.mark.parametrize("kw", [{"blinker": True}, {"desire": True}, {"angle": 45.0}, {"angle": -45.0}])
def test_a_turn_at_the_stop_keeps_the_usual_floor(kw):
  p = stopped(**kw)
  for v in (0.4, 1.0, 2.0):
    assert step(p, v) == USUAL  # kept after the blinker clicks off (kw only at the stop)


def test_wheel_near_straight_at_the_stop_is_not_a_turn():
  assert step(stopped(angle=9.1), 0.5) == pytest.approx(STRAIGHT_FLOOR)  # the stop in 00000097--40 +42 s sat at 9.1 deg


def test_the_wheel_winding_during_the_pull_away_does_not_switch_back():
  """The model's leftover request may wind the wheel; that must not bring the full request (the jerk) back."""
  p = stopped()
  for v, angle in ((0.4, 10.0), (0.7, 35.0), (1.0, 70.0), (1.5, 40.0)):
    assert step(p, v, angle=angle) == pytest.approx(STRAIGHT_FLOOR)


def test_a_brief_press_does_not_count_a_held_one_does():
  p = stopped()
  n_brief = round(PRESS_HOLD_S / DT) - 1
  for _ in range(n_brief):
    assert step(p, 0.6, pressed=True) == pytest.approx(STRAIGHT_FLOOR)
  assert step(p, 0.6) == pytest.approx(STRAIGHT_FLOOR)
  for _ in range(n_brief + 1):
    step(p, 0.6, pressed=True)
  assert step(p, 0.6) < STRAIGHT_FLOOR - 0.01  # blending toward the usual floor


def test_a_blinker_during_the_pull_away_blends_to_the_usual_floor_without_a_jump():
  p = stopped()
  v, lat_accel = 0.8, 0.3
  prev = curvature(lat_accel, v, step(p, v))
  frames = round(BLEND_S / DT)
  steps = []
  for _ in range(frames + 2):
    k = curvature(lat_accel, v, step(p, v, blinker=True))
    steps.append(k - prev)
    prev = k
  full = curvature(lat_accel, v, USUAL)
  assert prev == pytest.approx(full)
  assert max(steps) == pytest.approx((full - curvature(lat_accel, v, STRAIGHT_FLOOR)) / frames)  # even steps
  assert step(p, 1.2) == USUAL  # and stays for the rest of the pull-away


def test_stopping_again_decides_afresh():
  p = stopped(blinker=True)
  assert step(p, 0.5) == USUAL
  assert step(p, 0.0) == pytest.approx(STRAIGHT_FLOOR)  # stopped, nothing says turn now
  assert step(p, 0.4) == pytest.approx(STRAIGHT_FLOOR)


def test_switching_the_setting_mid_pull_away_never_jumps():
  p = stopped()
  assert step(p, 0.6) == pytest.approx(STRAIGHT_FLOOR)
  floors = [step(p, 0.6, enabled=False) for _ in range(round(BLEND_S / DT) + 1)]
  assert floors[0] < STRAIGHT_FLOOR
  assert floors[-1] == USUAL
  assert all(a >= b for a, b in zip(floors, floors[1:], strict=False))
  p = StraightPullAway()
  assert step(p, 0.6, enabled=True) == USUAL  # switched on while moving: waits for the next stop


def test_blend_floor_is_never_below_the_speed():
  p = stopped()
  for v in (0.3, 0.9, 1.4, 2.2):
    p.usual, p.target = 0.5, 1.0
    f = step(p, v)
    assert f >= v
    want = (0.5 + 0.1) * curvature(1.0, v, USUAL) + (0.5 - 0.1) * curvature(1.0, v, STRAIGHT_FLOOR)  # one frame on
    assert curvature(1.0, v, f) == pytest.approx(want)


def test_with_calmer_crawl_speed_steering_on_nothing_changes():
  p = StraightPullAway()
  for v in (0.0, 0.5, 2.0, 5.0):
    assert step(p, v, base=STRAIGHT_FLOOR, blinker=True) == STRAIGHT_FLOOR
    assert step(p, v, base=STRAIGHT_FLOOR) == STRAIGHT_FLOOR


def test_cheddars_standstill_bias_no_longer_pulls_the_wheel():
  """The numbers from 00000097--40: model lateral accel -0.026 at a standstill."""
  bias = -0.026
  p = stopped()
  for v in (0.35, 0.6, 1.0):
    assert abs(curvature(bias, v, step(p, v))) <= 0.0042 < abs(curvature(bias, v, USUAL))


def test_a_blinker_turn_from_a_stop_is_untouched():
  p = stopped(blinker=True)
  for v in (0.35, 0.6, 1.0, 2.0):
    assert curvature(0.30, v, step(p, v)) == curvature(0.30, v, USUAL)


def test_nan_inputs_do_not_raise_or_return_nan():
  p = StraightPullAway()
  assert step(p, 0.0, angle=float('nan')) == USUAL
  for v in (float('nan'), 0.5):
    f = step(p, v)
    assert math.isfinite(f)
    assert f >= USUAL
