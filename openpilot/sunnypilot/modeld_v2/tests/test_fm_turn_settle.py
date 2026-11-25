"""FM Pilot: Settle After Turn (FM_TurnSettle)."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.modeld_v2.fm_turn_settle import (TurnSettle, settle_from_model, MAX_CURVATURE, WINDOW, DT,
                                                           MAX_SPEED, HANDOVER_TRUST)
from openpilot.sunnypilot.modeld_v2.tests.test_blendful import frame

V = 6.7  # m/s, 15 mph


def output(center=0.0, probs=(0.35, 0.35), stds=(0.5, 0.5), edges=(math.inf, math.inf), edge_y=(-6.0, 6.0)):
  out = frame(center=center, probs=probs, stds=stds)
  out['lane_lines_stds'][0, 1, :, 0], out['lane_lines_stds'][0, 2, :, 0] = stds
  re = np.zeros((1, 2, 33, 2))
  re[0, 0, :, 0], re[0, 1, :, 0] = edge_y
  rs = np.ones((1, 2, 33, 2))
  rs[0, 0, :, 0], rs[0, 1, :, 0] = edges
  out['road_edges'], out['road_edges_stds'] = re, rs
  return out


def sm(curvature=0.0, lat=True, pressed=False):
  return {'controlsState': SimpleNamespace(curvature=curvature), 'carControl': SimpleNamespace(latActive=lat),
          'carState': SimpleNamespace(steeringPressed=pressed)}


def step(s, desire=log.Desire.none, v=V, out=None, curvature=0.0, pressed=False, trust=0.0, enabled=True, lane_change=False,
         lat=True):
  return settle_from_model(s, enabled, desire, log.LaneChangeState.laneChangeStarting if lane_change else log.LaneChangeState.off,
                           v, sm(curvature, lat, pressed), trust, out if out is not None else output())


def turn(s, desire=log.Desire.turnRight, degrees=90.0, v=4.0):
  """A blinker turn: the truck turns `degrees` toward it while the desire is on, then the blinker goes off."""
  sign = 1 if desire == log.Desire.turnRight else -1
  k = sign * 0.1
  frames = int(round(math.radians(degrees) / (abs(k) * v * DT)))
  for _ in range(frames):
    step(s, desire, v=v, curvature=k)


def run(s, seconds, **kw):
  return [step(s, **kw) for _ in range(int(round(seconds / DT)))]


class TestTurnSettle:
  def test_off_is_exactly_zero(self):
    s = TurnSettle()
    turn(s)
    assert all(step(s, enabled=False, out=output(0.8)) == 0.0 for _ in range(100))

  def test_not_after_a_turn_does_nothing(self):
    s = TurnSettle()
    assert all(k == 0.0 for k in run(s, 5.0, out=output(0.8)))

  def test_settles_right_after_a_right_turn(self):
    s = TurnSettle()
    turn(s)
    ks = run(s, 3.0, out=output(0.8))
    assert ks[-1] > 0.0
    assert max(ks) <= MAX_CURVATURE + 1e-9
    assert s.offset == pytest.approx(0.8, abs=0.05)

  def test_right_turn_centers_both_ways(self):
    s = TurnSettle()
    turn(s)
    assert run(s, 3.0, out=output(-0.8))[-1] < 0.0

  def test_small_turn_does_nothing(self):
    s = TurnSettle()
    turn(s, degrees=30.0)
    assert all(k == 0.0 for k in run(s, 3.0, out=output(0.8)))

  def test_left_turn_only_pushes_right(self):
    s = TurnSettle()
    turn(s, log.Desire.turnLeft)
    assert all(k >= 0.0 for k in run(s, 3.0, out=output(-0.8)))
    s = TurnSettle()
    turn(s, log.Desire.turnLeft)
    assert run(s, 3.0, out=output(0.8))[-1] > 0.0

  def test_deadband(self):
    s = TurnSettle()
    turn(s)
    assert all(k == 0.0 for k in run(s, 3.0, out=output(0.2)))

  def test_unsure_lines_ignored(self):
    s = TurnSettle()
    turn(s)
    assert all(k == 0.0 for k in run(s, 3.0, out=output(0.8, probs=(0.2, 0.2))))
    s = TurnSettle()
    turn(s)
    assert all(k == 0.0 for k in run(s, 3.0, out=output(0.8, stds=(1.2, 1.2))))

  def test_one_line_is_enough(self):
    s = TurnSettle()
    turn(s)
    # only the left line, at -0.95: 1.6 m in from it is +0.65
    assert run(s, 3.0, out=output(0.8, probs=(0.35, 0.05)))[-1] > 0.0

  def test_road_edge_only_limits_the_aim(self):
    s = TurnSettle()
    turn(s)
    # lane center 0.8 m right, but a sure curb 1.9 m right: never aim closer than 1.4 m to it, so at most 0.5 m over
    ks = run(s, 3.0, out=output(0.8, edges=(math.inf, 0.3), edge_y=(-6.0, 1.9)))
    assert s.offset == pytest.approx(0.5, abs=0.02)
    free = TurnSettle()
    turn(free)
    assert max(ks) < max(run(free, 3.0, out=output(0.8)))
    # a sure curb 1.5 m right: the aim (0.1 m) is inside the deadband
    s = TurnSettle()
    turn(s)
    assert all(k == 0.0 for k in run(s, 3.0, out=output(0.8, edges=(math.inf, 0.3), edge_y=(-6.0, 1.5))))
    # an unsure curb is ignored
    s = TurnSettle()
    turn(s)
    assert run(s, 3.0, out=output(0.8, edges=(math.inf, 0.8), edge_y=(-6.0, 1.9)))[-1] > 0.0

  def test_steering_lets_go_at_once(self):
    s = TurnSettle()
    turn(s)
    run(s, 2.0, out=output(0.8))
    assert step(s, out=output(0.8), pressed=True) == 0.0
    assert all(k == 0.0 for k in run(s, 2.0, out=output(0.8)))  # the window is over

  @pytest.mark.parametrize("kw", [{"trust": HANDOVER_TRUST}, {"v": MAX_SPEED + 0.5}, {"lane_change": True},
                                  {"desire": log.Desire.turnLeft}, {"lat": False}])
  def test_hands_over(self, kw):
    s = TurnSettle()
    turn(s)
    run(s, 2.0, out=output(0.8))
    before = s.curvature
    assert before > 0.0
    after = run(s, 2.0, out=output(0.8), **kw)
    assert abs(after[-1]) < before
    assert not s.active

  def test_window_ends(self):
    s = TurnSettle()
    turn(s)
    run(s, WINDOW + 0.5, out=output(0.8))
    assert not s.active
    assert run(s, 2.0, out=output(0.8))[-1] == 0.0

  def test_below_8_mph_waits(self):
    s = TurnSettle()
    turn(s)
    assert all(k == 0.0 for k in run(s, 2.0, out=output(0.8), v=3.0))
    assert run(s, 2.0, out=output(0.8), v=V)[-1] > 0.0

  def test_builds_gently(self):
    s = TurnSettle()
    turn(s)
    ks = np.array(run(s, 3.0, out=output(1.0)))
    assert np.max(np.diff(ks)) * V ** 2 / DT <= 0.5 + 1e-6  # lateral jerk at most 0.5 m/s^3

  def test_no_road_edges_in_the_output(self):
    s = TurnSettle()
    turn(s)
    out = output(0.8)
    del out['road_edges'], out['road_edges_stds']
    assert run(s, 3.0, out=out)[-1] > 0.0


def test_off_leaves_blendful_exactly():
  """modeld_v2 adds Settle After Turn to Blendful's correction; off it must add exactly nothing."""
  from openpilot.sunnypilot.modeld_v2.blendful import Blendful
  a, b, s = Blendful(), Blendful(), TurnSettle()
  turn(s)
  rng = np.random.default_rng(0)
  for _ in range(300):
    out = output(float(rng.uniform(-1, 1)), probs=(0.9, 0.9), stds=(0.1, 0.1))
    k = float(rng.uniform(-0.01, 0.01))
    plain = a.update(out, 20.0, True, k)
    combined = b.update(out, 20.0, True, k) + step(s, out=out, v=20.0, enabled=False)
    assert plain == combined
