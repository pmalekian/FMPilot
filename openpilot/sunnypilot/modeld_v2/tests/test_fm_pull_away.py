"""FM Pilot: FM_StraightPullAwayFloor (Calmer Straight Pull-Away): the decision, and what it does to the request."""
import numpy as np
import pytest

from openpilot.sunnypilot.modeld_v2.fm_pull_away import HANDOVER_SPEED, STRAIGHT_FLOOR, StraightPullAway

USUAL = 1.0


def step(p, v, enabled=True, base=USUAL, blinker=False, desire=False, angle=0.0, pressed=False):
  return p.update(enabled, base, v, blinker, desire, angle, pressed)


def curvature(lat_accel, v_ego, floor):
  return lat_accel / max(floor, v_ego) ** 2


def test_off_is_the_usual_floor_always():
  p = StraightPullAway()
  for v in (0.0, 0.5, 1.0, 2.0, 5.0):
    assert step(p, v, enabled=False) == USUAL


def test_straight_pull_away_uses_the_calmer_floor():
  p = StraightPullAway()
  assert step(p, 0.0) == STRAIGHT_FLOOR
  for v in (0.31, 0.7, 1.0, 1.8, 2.4):
    assert step(p, v) == STRAIGHT_FLOOR
  assert step(p, 2.6) == USUAL  # both floors agree from here
  assert curvature(0.5, 2.6, STRAIGHT_FLOOR) == curvature(0.5, 2.6, USUAL)


@pytest.mark.parametrize("kw", [{"blinker": True}, {"desire": True}, {"pressed": True}, {"angle": 45.0}, {"angle": -45.0}])
def test_any_sign_of_a_turn_keeps_the_usual_floor(kw):
  p = StraightPullAway()
  assert step(p, 0.0, **kw) == USUAL
  assert step(p, 0.6, **kw) == USUAL


def test_wheel_near_straight_is_not_a_turn():
  p = StraightPullAway()
  assert step(p, 0.0, angle=9.1) == STRAIGHT_FLOOR  # the stop in 00000097--40 +42 s sat at 9.1 deg
  assert step(p, 0.5, angle=29.9) == STRAIGHT_FLOOR


def test_a_turn_stays_a_turn_after_the_signal_cancels_until_the_handover_speed():
  p = StraightPullAway()
  assert step(p, 0.4, blinker=True) == USUAL
  # blinker off and desire over at 1 m/s: the request must not drop mid-turn
  for v in (0.8, 1.2, 2.0, 2.4):
    assert step(p, v) == USUAL
  assert step(p, HANDOVER_SPEED + 0.1) == USUAL
  # a later crawl starts afresh
  assert step(p, 1.0) == STRAIGHT_FLOOR


def test_stopping_forgets_the_turn():
  p = StraightPullAway()
  assert step(p, 0.5, blinker=True) == USUAL
  assert step(p, 0.0) == STRAIGHT_FLOOR  # stopped, nothing says turn: the next pull-away is straight again
  assert step(p, 0.4) == STRAIGHT_FLOOR


def test_a_blinker_at_a_standstill_is_taken_into_the_pull_away():
  p = StraightPullAway()
  assert step(p, 0.0, blinker=True) == USUAL
  assert step(p, 0.4) == USUAL  # blinker off at the same moment: still the turn
  assert step(p, 1.5) == USUAL


def test_a_blinker_during_the_crawl_switches_to_the_usual_floor_and_stays():
  p = StraightPullAway()
  assert step(p, 0.5) == STRAIGHT_FLOOR
  assert step(p, 0.9, blinker=True) == USUAL
  assert step(p, 1.3) == USUAL


def test_with_calmer_crawl_speed_steering_on_nothing_changes():
  p = StraightPullAway()
  for v in (0.0, 0.5, 2.0, 5.0):
    assert step(p, v, base=STRAIGHT_FLOOR, blinker=True) == STRAIGHT_FLOOR
    assert step(p, v, base=STRAIGHT_FLOOR) == STRAIGHT_FLOOR


def test_cheddars_standstill_bias_no_longer_pulls_the_wheel():
  """The numbers from 00000097--40: model lateral accel -0.026 at a standstill."""
  bias = -0.026
  p = StraightPullAway()
  usual = [curvature(bias, v, USUAL) for v in (0.35, 0.6, 1.0)]
  calm = [curvature(bias, v, step(p, v)) for v in (0.35, 0.6, 1.0)]
  assert min(usual) == pytest.approx(-0.026)
  assert max(abs(np.array(calm))) <= 0.0042
  assert all(abs(c) < abs(u) for c, u in zip(calm, usual, strict=True))


def test_a_real_turn_from_a_stop_is_untouched():
  p = StraightPullAway()
  asked = 0.30  # a firm model request from a stop, blinker on
  for v in (0.35, 0.6, 1.0, 2.0):
    assert curvature(asked, v, step(p, v, blinker=True)) == curvature(asked, v, USUAL)


def test_nan_and_garbage_inputs_do_not_raise():
  p = StraightPullAway()
  assert step(p, float('nan')) in (USUAL, STRAIGHT_FLOOR)
  assert step(p, 0.5, angle=float('nan')) in (USUAL, STRAIGHT_FLOOR)
