from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_desire import FmTurnDesire, TurnDirection, PARAM_ENABLED, PARAM_MAX_SPEED


class FakeParams:
  def __init__(self, enabled=True, max_speed=45.0):
    self.values = {PARAM_ENABLED: enabled, PARAM_MAX_SPEED: max_speed}

  def get_bool(self, key):
    return bool(self.values.get(key))

  def get(self, key, return_default=False):
    return self.values.get(key)


def cs(mph, left=False, right=False, pressed=False, torque=0.0, left_bsm=False, right_bsm=False):
  return SimpleNamespace(vEgo=mph * CV.MPH_TO_MS, leftBlinker=left, rightBlinker=right, steeringPressed=pressed,
                         steeringTorque=torque, leftBlindspot=left_bsm, rightBlindspot=right_bsm)


def step(t, carstate, stock=TurnDirection.none, left_edge=False, right_edge=False, lat=True):
  t.update(carstate, lat, stock, left_edge, right_edge)
  return t.turn_direction


NUDGE_RIGHT = {"pressed": True, "torque": -1.0}


class TestFmTurnDesire:
  def test_disabled_does_nothing(self):
    t = FmTurnDesire(FakeParams(enabled=False))
    assert step(t, cs(35, right=True, **NUDGE_RIGHT), right_edge=True) == TurnDirection.none
    assert not t.armed

  def test_edge_blinker_starts_turn_without_nudge(self):
    t = FmTurnDesire(FakeParams())
    assert step(t, cs(35, right=True), right_edge=True) == TurnDirection.turnRight

  def test_no_edge_no_turn_above_20(self):
    t = FmTurnDesire(FakeParams())
    assert step(t, cs(35, right=True)) == TurnDirection.none
    assert not t.armed

  def test_edge_turn_needs_lateral_active(self):
    t = FmTurnDesire(FakeParams())
    assert step(t, cs(35, right=True), right_edge=True, lat=False) == TurnDirection.none

  def test_slow_turn_starts_while_stopped(self):
    # openpilot never steers at a standstill; AP's Lane Turn Desire starts anyway, so FM Pilot's must too
    t = FmTurnDesire(FakeParams())
    assert step(t, cs(0, left=True), stock=TurnDirection.turnLeft, lat=False) == TurnDirection.turnLeft

  def test_blinker_cycle_while_stopped_restarts_turn(self):
    t = FmTurnDesire(FakeParams())
    step(t, cs(0, right=True), stock=TurnDirection.turnRight, lat=False)
    assert step(t, cs(0), lat=False) == TurnDirection.none
    assert step(t, cs(0, right=True), stock=TurnDirection.turnRight, lat=False) == TurnDirection.turnRight

  def test_held_through_slowdown_and_stop(self):
    t = FmTurnDesire(FakeParams())
    step(t, cs(35, left=True), left_edge=True)
    for mph in (25, 15, 5, 0, 0, 3):
      assert step(t, cs(mph, left=True)) == TurnDirection.turnLeft

  def test_held_when_edge_disappears(self):
    t = FmTurnDesire(FakeParams())
    step(t, cs(35, right=True), right_edge=True)
    assert step(t, cs(30, right=True)) == TurnDirection.turnRight

  def test_under_20_follows_stock_arming(self):
    t = FmTurnDesire(FakeParams())
    assert step(t, cs(12, right=True)) == TurnDirection.none  # stock lane turn off or too fast
    assert step(t, cs(12, right=True), stock=TurnDirection.turnRight) == TurnDirection.turnRight

  def test_blinker_off_cancels(self):
    t = FmTurnDesire(FakeParams())
    step(t, cs(35, right=True), right_edge=True)
    assert step(t, cs(20)) == TurnDirection.none
    assert not t.armed

  def test_switched_blinker_cancels(self):
    t = FmTurnDesire(FakeParams())
    step(t, cs(35, right=True), right_edge=True)
    assert step(t, cs(35, left=True)) == TurnDirection.none

  def test_speed_ceiling(self):
    t = FmTurnDesire(FakeParams(max_speed=30.0))
    assert step(t, cs(40, right=True), right_edge=True) == TurnDirection.none
    assert step(t, cs(29, right=True), right_edge=True) == TurnDirection.turnRight

  def test_blindspot_blocks_turn(self):
    t = FmTurnDesire(FakeParams())
    assert step(t, cs(35, right=True, right_bsm=True), right_edge=True) == TurnDirection.none
    assert not t.armed


class TestDesireHelperIntegration:
  def make(self, enabled):
    from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    dh.alc.read_params = lambda: None
    dh.fm_turn.params = FakeParams(enabled=enabled)
    dh.fm_turn.read_params()
    return dh

  def test_disabled_is_stock_lane_change(self):
    from openpilot.cereal import log
    dh = self.make(False)
    dh.update(cs(35), True, 0.0)
    dh.update(cs(35, right=True), True, 0.0, False, True)
    assert dh.lane_change_state == log.LaneChangeState.preLaneChange
    assert dh.lane_turn_direction == TurnDirection.none

  def test_edge_turn_replaces_lane_change(self):
    from openpilot.cereal import log
    dh = self.make(True)
    dh.update(cs(35), True, 0.0)
    dh.update(cs(35, right=True), True, 0.0, False, True)
    assert dh.lane_change_state == log.LaneChangeState.off  # a turn, not a lane change prompt
    assert dh.desire == log.Desire.turnRight
    assert dh.lane_change_state == log.LaneChangeState.off
