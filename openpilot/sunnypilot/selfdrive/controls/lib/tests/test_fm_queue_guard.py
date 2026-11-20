import math
from types import SimpleNamespace

import pytest

from openpilot.cereal import log, custom
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.fm_queue_guard import FmQueueGuard, PARAM, LEAD_CLOSE, LEAD_CLEAR, QUEUE_SPEED

TurnDirection = custom.ModelDataV2SP.TurnDirection


class FakeParams:
  def __init__(self, enabled=True):
    self.values = {PARAM: enabled}

  def get_bool(self, key):
    return bool(self.values.get(key))

  def get(self, key, return_default=False):
    return self.values.get(key)


def step(g, v, lead, sign=-1, k=0.0, n=1):
  out = None
  for _ in range(n):
    out = g.update(sign, v, k, DT_MDL, lead is not None, lead if lead is not None else math.inf)
  return out


class TestFmQueueGuard:
  def test_off_never_holds(self):
    g = FmQueueGuard(FakeParams(False))
    assert not step(g, 0.0, 3.0, n=40)

  def test_holds_when_stopped_close_behind_a_car(self):
    g = FmQueueGuard(FakeParams())
    assert step(g, 0.0, 3.0)

  def test_does_not_start_while_moving(self):
    g = FmQueueGuard(FakeParams())
    assert not step(g, 1.0, 3.0, n=20)  # creeping up behind it, not yet stopped
    assert step(g, 0.0, 3.0)

  def test_far_lead_or_none_never_holds(self):
    g = FmQueueGuard(FakeParams())
    assert not step(g, 0.0, LEAD_CLOSE + 0.5, n=20)
    assert not step(g, 0.0, None, n=20)

  def test_keeps_holding_through_a_creep_then_lets_go(self):
    """000000af--3: stopped 2.4-3.6 m behind, creeps at 1-5 mph with the car 4-9 m ahead, it pulls away."""
    g = FmQueueGuard(FakeParams())
    assert step(g, 0.0, 2.5, n=20)
    assert step(g, 1.5, 5.4, n=20)  # creep, car still close
    assert step(g, 0.0, 3.8, n=20)
    assert step(g, 1.0, 8.9)  # car pulling away, still within 10 m
    assert not step(g, 1.5, LEAD_CLEAR + 0.5)  # more than 10 m: the turn comes back
    assert not step(g, 1.5, 14.5, n=20)

  def test_lets_go_at_10_mph(self):
    g = FmQueueGuard(FakeParams())
    assert step(g, 0.0, 3.0)
    assert not step(g, QUEUE_SPEED + 0.01, 3.0)

  def test_car_gone_lets_go(self):
    g = FmQueueGuard(FakeParams())
    assert step(g, 0.0, 3.0)
    assert not step(g, 0.0, None)

  def test_keeps_holding_through_a_fast_creep(self):
    """000000af--3 +22.3 s: a 5.5 mph creep with the car 6-7 m ahead, which slowed again."""
    g = FmQueueGuard(FakeParams())
    assert step(g, 0.0, 3.0)
    assert step(g, 5.5 * 0.44704, 6.5, n=10)

  def test_lets_go_when_you_steer_the_turn(self):
    g = FmQueueGuard(FakeParams())
    assert step(g, 0.0, 3.0)
    assert not step(g, 2.0, 4.0, k=-0.1, n=20)  # 2 m/s at 10 m radius toward the left turn for 1 s: 11 degrees

  @pytest.mark.parametrize("sign", [1, -1])
  def test_never_holds_a_turn_already_under_way(self, sign):
    g = FmQueueGuard(FakeParams())
    step(g, 3.0, None, sign=sign, k=sign * 0.1, n=20)  # 3 m/s at 10 m radius for 1 s: about 17 degrees turned
    assert not step(g, 0.0, 3.0, sign=sign, n=20)  # stops behind a car mid-turn: keeps going

  def test_turning_the_other_way_does_not_count(self):
    g = FmQueueGuard(FakeParams())
    step(g, 3.0, None, sign=-1, k=0.1, n=20)  # a left turn desire while the truck turns right
    assert step(g, 0.0, 3.0, sign=-1)

  def test_no_turn_resets(self):
    g = FmQueueGuard(FakeParams())
    assert step(g, 0.0, 3.0)
    assert not step(g, 0.0, 3.0, sign=0)
    assert g.turned == 0.0


def cs(v, left=False):
  return SimpleNamespace(vEgo=v, leftBlinker=left, rightBlinker=False, steeringPressed=False, steeringTorque=0.0,
                         leftBlindspot=False, rightBlindspot=False, brakePressed=False)


class TestDesireHelperQueueGuard:
  def make(self, enabled):
    from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
    dh = DesireHelper()
    dh.lane_turn_controller.enabled = True
    dh.lane_turn_controller.read_params = lambda: None
    dh.fm_queue_guard.params = FakeParams(enabled=enabled)
    dh.fm_queue_guard.read_params()
    return dh

  @staticmethod
  def lead(d):
    return SimpleNamespace(present=d is not None, dRel=d if d is not None else 0.0)

  def test_off_turn_stays(self):
    dh = self.make(False)
    dh.update(cs(0.0, left=True), False, 0.0, lead=self.lead(3.0))
    assert dh.lane_turn_direction == TurnDirection.turnLeft
    assert dh.desire == log.Desire.turnLeft
    assert not dh.fm_queue_hold

  def test_on_turn_waits_then_comes_back_fresh(self):
    dh = self.make(True)
    dh.update(cs(0.0, left=True), False, 0.0, lead=self.lead(3.0))
    assert dh.fm_queue_hold
    assert dh.lane_turn_direction == TurnDirection.none
    assert dh.desire == dh.model_desire == log.Desire.none
    dh.update(cs(1.0, left=True), True, 0.0, lead=self.lead(12.0))
    assert not dh.fm_queue_hold
    assert dh.lane_turn_direction == TurnDirection.turnLeft
    assert dh.desire == log.Desire.turnLeft

  def test_no_lead_message_never_holds(self):
    dh = self.make(True)
    dh.update(cs(0.0, left=True), False, 0.0)
    assert dh.lane_turn_direction == TurnDirection.turnLeft
