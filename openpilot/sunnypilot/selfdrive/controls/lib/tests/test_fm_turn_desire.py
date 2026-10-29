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


def step(t, carstate, stock=TurnDirection.none, left_edge=False, right_edge=False, mode=AutoLaneChangeMode.NUDGE, lat=True):
  t.update(carstate, lat, stock, left_edge, right_edge, mode)
  return t.turn_direction


NUDGE_RIGHT = {"pressed": True, "torque": -1.0}


class TestFmTurnDesire:
  def test_disabled_does_nothing(self):
    t = FmTurnDesire(FakeParams(enabled=False))
    assert step(t, cs(35, right=True, **NUDGE_RIGHT), right_edge=True) == TurnDirection.none
    assert not t.armed

  def test_edge_arms_then_nudge_starts(self):
    t = FmTurnDesire(FakeParams())
    assert step(t, cs(35, right=True), right_edge=True) == TurnDirection.none
    assert t.armed
    assert step(t, cs(35, right=True, **NUDGE_RIGHT), right_edge=True) == TurnDirection.turnRight

  def test_no_edge_no_arm_above_20(self):
    t = FmTurnDesire(FakeParams())
    step(t, cs(35, right=True))
    assert not t.armed

  def test_nudge_wrong_way_does_not_start(self):
    t = FmTurnDesire(FakeParams())
    step(t, cs(35, right=True), right_edge=True)
    assert step(t, cs(35, right=True, pressed=True, torque=1.0), right_edge=True) == TurnDirection.none

  def test_held_through_slowdown_and_stop(self):
    t = FmTurnDesire(FakeParams())
    step(t, cs(35, left=True), left_edge=True)
    step(t, cs(35, left=True, pressed=True, torque=1.0), left_edge=True)
    for mph in (25, 15, 5, 0, 0, 3):
      assert step(t, cs(mph, left=True)) == TurnDirection.turnLeft

  def test_slowing_below_20_still_needs_nudge(self):
    # signalled at 40, edge seen, slowed to 15 before the turn: the stock lane-turn arming does not start the turn
    t = FmTurnDesire(FakeParams())
    step(t, cs(40, right=True), right_edge=True)
    assert step(t, cs(15, right=True), stock=TurnDirection.turnRight) == TurnDirection.none
    assert step(t, cs(15, right=True, **NUDGE_RIGHT), stock=TurnDirection.turnRight) == TurnDirection.turnRight

  def test_under_20_follows_stock_arming(self):
    t = FmTurnDesire(FakeParams())
    assert step(t, cs(12, right=True)) == TurnDirection.none and not t.armed  # stock lane turn off or too fast
    assert step(t, cs(12, right=True), stock=TurnDirection.turnRight) == TurnDirection.none and t.armed
    assert step(t, cs(12, right=True, **NUDGE_RIGHT), stock=TurnDirection.turnRight) == TurnDirection.turnRight

  def test_blinker_off_cancels(self):
    t = FmTurnDesire(FakeParams())
    step(t, cs(35, right=True), right_edge=True)
    step(t, cs(35, right=True, **NUDGE_RIGHT), right_edge=True)
    assert step(t, cs(20)) == TurnDirection.none
    assert not t.armed

  def test_switched_blinker_cancels(self):
    t = FmTurnDesire(FakeParams())
    step(t, cs(35, right=True), right_edge=True)
    step(t, cs(35, right=True, **NUDGE_RIGHT), right_edge=True)
    assert step(t, cs(35, left=True)) == TurnDirection.none

  def test_speed_ceiling(self):
    t = FmTurnDesire(FakeParams(max_speed=30.0))
    step(t, cs(40, right=True), right_edge=True)
    assert not t.armed
    step(t, cs(29, right=True), right_edge=True)
    assert t.armed

  def test_blindspot_blocks_arming(self):
    t = FmTurnDesire(FakeParams())
    step(t, cs(35, right=True, right_bsm=True), right_edge=True)
    assert not t.armed

  def test_automatic_lane_change_mode_starts_without_nudge(self):
    t = FmTurnDesire(FakeParams())
    assert step(t, cs(35, right=True), right_edge=True, mode=AutoLaneChangeMode.NUDGELESS) == TurnDirection.turnRight


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
    assert dh.lane_change_state == log.LaneChangeState.off  # armed: no lane change prompt
    assert dh.desire == log.Desire.none
    dh.update(cs(35, right=True, **NUDGE_RIGHT), True, 0.0, False, True)
    assert dh.desire == log.Desire.turnRight
    assert dh.lane_change_state == log.LaneChangeState.off
