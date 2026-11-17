from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.fm_nav_turn_desire import FmNavTurnDesire, TurnDirection, PARAM_ENABLED, start_distance

LaneChangeState = log.LaneChangeState
NO_PICK = SimpleNamespace(valid=False, distance=0.0)


class FakeParams:
  def __init__(self, enabled=True):
    self.enabled = enabled

  def get_bool(self, key):
    return self.enabled if key == PARAM_ENABLED else False


def cs(mph, left=False, right=False, left_bsm=False, right_bsm=False):
  return SimpleNamespace(vEgo=mph * CV.MPH_TO_MS, leftBlinker=left, rightBlinker=right, steeringPressed=False, steeringTorque=0.0,
                         leftBlindspot=left_bsm, rightBlindspot=right_bsm, brakePressed=False)


def nav(left=None, right=None, status="ok"):
  pick = lambda d: NO_PICK if d is None else SimpleNamespace(valid=True, distance=d)  # noqa: E731
  return SimpleNamespace(status=status, left=pick(left), right=pick(right))


def step(t, carstate, n, lat=True, lane_changing=False):
  t.update(carstate, lat, n, lane_changing)
  return t.turn_direction


class TestFmNavTurnDesire:
  def test_off_does_nothing(self):
    t = FmNavTurnDesire(FakeParams(enabled=False))
    assert step(t, cs(25, right=True), nav(right=30)) == TurnDirection.none
    assert not t.armed and not t.active

  def test_street_on_the_blinker_side_starts_the_turn(self):
    t = FmNavTurnDesire(FakeParams())
    assert step(t, cs(25, right=True), nav(right=40)) == TurnDirection.turnRight
    t = FmNavTurnDesire(FakeParams())
    assert step(t, cs(25, left=True), nav(left=40)) == TurnDirection.turnLeft

  def test_street_on_the_other_side_does_nothing(self):
    t = FmNavTurnDesire(FakeParams())
    assert step(t, cs(25, right=True), nav(left=40)) == TurnDirection.none
    assert not t.armed

  def test_far_junction_arms_then_starts(self):
    t = FmNavTurnDesire(FakeParams())
    v = 22
    far = start_distance(v * CV.MPH_TO_MS) + 5
    assert step(t, cs(v, right=True), nav(right=far)) == TurnDirection.none
    assert t.armed  # holds off the lane change
    assert step(t, cs(v, right=True), nav(right=far - 10)) == TurnDirection.turnRight

  def test_start_distance_limits(self):
    assert start_distance(1.0) == 35.0 and start_distance(30.0) == 80.0
    assert start_distance(10.0) == 60.0

  def test_no_blinker_no_turn(self):
    t = FmNavTurnDesire(FakeParams())
    assert step(t, cs(25), nav(left=30, right=30)) == TurnDirection.none
    assert not t.armed

  def test_speed_window(self):
    for mph in (19, 36, 60):
      t = FmNavTurnDesire(FakeParams())
      assert step(t, cs(mph, right=True), nav(right=30)) == TurnDirection.none, mph
      assert not t.armed
    for mph in (21, 34):
      t = FmNavTurnDesire(FakeParams())
      assert step(t, cs(mph, right=True), nav(right=30)) == TurnDirection.turnRight, mph

  def test_held_until_the_blinker_is_cancelled(self):
    t = FmNavTurnDesire(FakeParams())
    step(t, cs(25, right=True), nav(right=40))
    # slower than the window, the map gone, openpilot not steering: still the turn
    assert step(t, cs(5, right=True), None, lat=False) == TurnDirection.turnRight
    assert step(t, cs(0, right=True), nav(status="noFix")) == TurnDirection.turnRight
    assert step(t, cs(5), None) == TurnDirection.none
    assert not t.active and not t.armed

  def test_switched_blinker_ends_it(self):
    t = FmNavTurnDesire(FakeParams())
    step(t, cs(25, right=True), nav(right=40))
    assert step(t, cs(25, left=True), nav(right=40)) == TurnDirection.none

  def test_needs_openpilot_steering_and_a_clear_blind_spot(self):
    t = FmNavTurnDesire(FakeParams())
    assert step(t, cs(25, right=True), nav(right=40), lat=False) == TurnDirection.none
    assert t.armed
    assert step(t, cs(25, right=True, right_bsm=True), nav(right=40)) == TurnDirection.none
    assert step(t, cs(25, right=True), nav(right=40)) == TurnDirection.turnRight

  def test_no_map_message_or_a_bad_status(self):
    for n in (None, nav(right=40, status="offRoad"), nav(right=40, status="noFix"), nav(right=40, status="tooFast")):
      t = FmNavTurnDesire(FakeParams())
      assert step(t, cs(25, right=True), n) == TurnDirection.none
      assert not t.armed

  def test_a_blinker_that_changed_lanes_never_becomes_a_turn(self):
    # Nov 3 log, --50 +5.5 s: lane change at 42 mph, blinker left on while braking for a light, a street 80 m ahead
    t = FmNavTurnDesire(FakeParams())
    assert step(t, cs(42, left=True), nav(), lane_changing=True) == TurnDirection.none
    assert step(t, cs(34, left=True), nav(left=70)) == TurnDirection.none
    assert not t.armed
    step(t, cs(30), nav(left=40))  # cancelled
    assert step(t, cs(30, left=True), nav(left=40)) == TurnDirection.turnLeft  # a new blinker is a new question

  def test_turned_on_while_driving(self):
    p = FakeParams(enabled=False)
    t = FmNavTurnDesire(p)
    p.enabled = True
    for _ in range(60):
      out = step(t, cs(25, right=True), nav(right=40))
    assert out == TurnDirection.turnRight


def dh_cs(mph, **kw):
  return cs(mph, **kw)


class TestDesireHelperIntegration:
  def make(self, enabled):
    from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
    dh = DesireHelper()
    dh.lane_turn_controller.enabled = True
    dh.lane_turn_controller.read_params = lambda: None
    dh.alc.read_params = lambda: None
    dh.fm_turn.params = FakeParams(enabled=False)
    dh.fm_turn.read_params = lambda: None
    dh.fm_turn.enabled = False
    dh.fm_nav_turn.params = FakeParams(enabled=enabled)
    dh.fm_nav_turn.read_params()
    return dh

  def test_mapped_street_is_a_turn_not_a_lane_change(self):
    dh = self.make(True)
    dh.update(dh_cs(25), True, 0.0, nav_turn=nav(right=40))
    dh.update(dh_cs(25, right=True), True, 0.0, nav_turn=nav(right=40))
    assert dh.lane_turn_direction == TurnDirection.turnRight
    assert dh.desire == log.Desire.turnRight
    assert dh.lane_change_state == LaneChangeState.off

  def test_armed_holds_off_the_lane_change(self):
    dh = self.make(True)
    dh.update(dh_cs(34), True, 0.0, nav_turn=nav(right=79))
    dh.update(dh_cs(34, right=True), True, 0.0, nav_turn=nav(right=100))  # beyond the start distance (80 m at most)
    assert dh.fm_nav_turn.armed and dh.desire == log.Desire.none
    assert dh.lane_change_state == LaneChangeState.off

  def test_off_is_the_lane_change_as_before(self):
    with_map, without = self.make(False), self.make(False)
    for carstate in (dh_cs(25), dh_cs(25, right=True), dh_cs(25, right=True)):
      with_map.update(carstate, True, 0.0, nav_turn=nav(right=40))
      without.update(carstate, True, 0.0)
      assert (with_map.desire, with_map.lane_change_state, with_map.lane_turn_direction) == \
             (without.desire, without.lane_change_state, without.lane_turn_direction)
    assert without.lane_change_state == LaneChangeState.preLaneChange

  def test_on_without_a_street_is_the_lane_change_as_before(self):
    dh = self.make(True)
    dh.update(dh_cs(25), True, 0.0, nav_turn=nav())
    dh.update(dh_cs(25, right=True), True, 0.0, nav_turn=nav())
    assert dh.lane_change_state == LaneChangeState.preLaneChange and dh.desire == log.Desire.none

  def test_slow_turns_stay_ap_lane_turn_desire(self):
    dh = self.make(True)
    dh.lane_turn_controller.lane_turn_value = 20 * CV.MPH_TO_MS
    dh.update(dh_cs(10, left=True), True, 0.0, nav_turn=nav())
    assert dh.lane_turn_direction == TurnDirection.turnLeft and not dh.fm_nav_turn.active
