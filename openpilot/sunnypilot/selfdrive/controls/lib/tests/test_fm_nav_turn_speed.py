from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel
from openpilot.sunnypilot.selfdrive.controls.lib.fm_nav_turn_speed import (FmNavTurnSpeed, TurnDirection, PARAM_ENABLED, PARAM_SPEED,
                                                                          TURN_IN_BEFORE, A_PLAN, CRUISE_LAG, turn_speed_limit)

MPH = CV.MPH_TO_MS
NONE, LEFT, RIGHT = TurnDirection.none, TurnDirection.turnLeft, TurnDirection.turnRight


class FakeParams:
  def __init__(self, enabled=True, mph=14):
    self.enabled, self.mph = enabled, mph

  def get_bool(self, key):
    return self.enabled if key == PARAM_ENABLED else False

  def get(self, key, return_default=False):
    return self.mph if key == PARAM_SPEED else None


def nav(left=None, right=None, heading=0.0, status="ok"):
  pick = lambda d: SimpleNamespace(valid=d is not None, distance=d or 0.0)  # noqa: E731
  return SimpleNamespace(status=status, heading=heading, left=pick(left), right=pick(right))


class TestNavTurnSpeed:
  def run(self, ts, t, v, turn, n, long_enabled=True, override=False, a=0.0):
    ts.update(t, long_enabled, override, v, a, turn, n, t)
    return ts.output_v_target

  def test_off_or_no_turn_or_no_street_never_caps(self):
    for ts, turn, n in ((FmNavTurnSpeed(FakeParams(enabled=False)), RIGHT, nav(right=50)),
                        (FmNavTurnSpeed(FakeParams()), NONE, nav(right=50)),
                        (FmNavTurnSpeed(FakeParams()), RIGHT, nav(left=50)),
                        (FmNavTurnSpeed(FakeParams()), RIGHT, nav(right=50, status="offRoad")),
                        (FmNavTurnSpeed(FakeParams()), RIGHT, None)):
      assert self.run(ts, 0.0, 13.0, turn, n) == V_CRUISE_UNSET and not ts.active

  def test_only_while_openpilot_controls_speed(self):
    for kw in ({"long_enabled": False}, {"override": True}):
      ts = FmNavTurnSpeed(FakeParams())
      assert self.run(ts, 0.0, 13.0, RIGHT, nav(right=50), **kw) == V_CRUISE_UNSET

  def test_cap_reaches_the_turn_speed_at_turn_in(self):
    ts = FmNavTurnSpeed(FakeParams(mph=14))
    cap = self.run(ts, 0.0, 13.4, RIGHT, nav(right=60))
    assert cap == pytest.approx(turn_speed_limit(60 - TURN_IN_BEFORE, 14 * MPH) - A_PLAN * CRUISE_LAG)
    assert 14 * MPH < cap < 13.4  # 30 mph is too fast 50 m from turn-in
    assert ts.output_a_target == 0.0  # the planner's own acceleration, handed back unchanged
    assert self.run(ts, 0.2, 7.0, RIGHT, nav(right=TURN_IN_BEFORE)) == pytest.approx(14 * MPH)
    assert self.run(ts, 0.4, 6.0, RIGHT, nav(right=0)) == pytest.approx(14 * MPH)  # through the corner

  def test_left_uses_the_left_street(self):
    ts = FmNavTurnSpeed(FakeParams())
    assert self.run(ts, 0.0, 10.0, LEFT, nav(left=30, right=80)) == pytest.approx(turn_speed_limit(30 - TURN_IN_BEFORE, 14 * MPH) - A_PLAN * CRUISE_LAG)

  def test_speed_setting(self):
    for mph, want in ((10, 10), (20, 20), (5, 10), (40, 20)):
      ts = FmNavTurnSpeed(FakeParams(mph=mph))
      assert self.run(ts, 0.0, 10.0, RIGHT, nav(right=TURN_IN_BEFORE)) == pytest.approx(want * MPH)

  def test_never_adds_speed_or_braking_when_slower(self):
    ts = FmNavTurnSpeed(FakeParams())
    a = 0.6
    ts.update(0.0, True, False, 2.0, a, RIGHT, nav(right=20), 0.0)  # pulling away from a stop at the corner
    assert ts.output_v_target >= 14 * MPH and ts.output_a_target == a

  def test_carried_forward_between_map_messages(self):
    ts = FmNavTurnSpeed(FakeParams())
    msg = nav(right=60)
    ts.update(0.0, True, False, 10.0, 0.0, RIGHT, msg, 0.0)
    for i in range(1, 4):  # 20 Hz planner, one map message at 0.0
      ts.update(0.05 * i, True, False, 10.0, 0.0, RIGHT, msg, 0.0)
    assert ts.distance == pytest.approx(60 - 3 * 0.5)

  def test_stale_map_doesnt_start_it(self):
    ts = FmNavTurnSpeed(FakeParams())
    ts.update(5.0, True, False, 10.0, 0.0, RIGHT, nav(right=40), 4.0)
    assert not ts.active

  def test_ends_when_turned(self):
    ts = FmNavTurnSpeed(FakeParams())
    self.run(ts, 0.0, 7.0, RIGHT, nav(right=15, heading=350.0))
    assert self.run(ts, 0.2, 6.0, RIGHT, nav(heading=30.0, status="offRoad")) != V_CRUISE_UNSET  # 40 deg, mid-turn, off the map
    assert self.run(ts, 0.4, 6.0, RIGHT, nav(heading=55.0)) == V_CRUISE_UNSET  # 65 deg: done
    assert not ts.active

  def test_ends_when_passed_without_turning(self):
    ts = FmNavTurnSpeed(FakeParams())
    self.run(ts, 0.0, 10.0, RIGHT, nav(right=5))
    # the next street to the right shows up 60 m on: it is not adopted
    t = 0.0
    while ts.active and t < 10:
      t += 0.05
      self.run(ts, t, 10.0, RIGHT, nav(right=60))
    assert not ts.active and 2.0 < t < 3.5  # 5 m + 20 m at 10 m/s

  def test_ends_with_the_turn_desire_or_a_switched_side(self):
    ts = FmNavTurnSpeed(FakeParams())
    self.run(ts, 0.0, 10.0, RIGHT, nav(right=40))
    assert self.run(ts, 0.05, 10.0, NONE, nav(right=40)) == V_CRUISE_UNSET
    self.run(ts, 0.1, 10.0, RIGHT, nav(right=40))
    assert self.run(ts, 0.15, 10.0, LEFT, nav(left=40)) == V_CRUISE_UNSET

  @pytest.mark.parametrize("mph0,start", [(30, 80), (25, 67), (22, 60), (18, 40)])
  def test_approach_with_the_planners_cruise_accel(self, mph0, start):
    """Point-mass truck on the planner's own cruise acceleration (limits and jerk); the turn is planned `start` m out,
    as Map Turn Desire does at that speed. Turn speed 14 mph."""
    CP = SimpleNamespace(carFingerprint="RIVIAN_R1_GEN1")
    ts, dt, v, a, x = FmNavTurnSpeed(FakeParams()), 0.05, mph0 * MPH, 0.0, 0.0
    rows = []
    for i in range(600):
      t = i * dt
      if i % 4 == 0:  # the map's 5 Hz
        msg, msg_t = nav(right=start - x), t
      ts.update(t, True, False, v, a, RIGHT, msg, msg_t)
      a = get_cruise_accel(True, min(mph0 * MPH, ts.output_v_target), v, a, 0.0, CP, dt, 0.0, True)
      v = max(v + a * dt, 0.0)
      x += v * dt
      rows.append((x - start, v, a))
      if x > start + 5:
        break
    d = np.array(rows)
    assert np.interp(-TURN_IN_BEFORE, d[:, 0], d[:, 1]) < 15.5 * MPH  # about 14 mph where the turn starts
    assert np.interp(0.0, d[:, 0], d[:, 1]) < 14.6 * MPH
    assert d[:, 2].min() >= -1.21  # never firmer than the planner's cruise limit


class TestInThePlanner:
  """The real LongitudinalPlanner (sunnypilot's targets min() and the cruise accel), mocked messages."""

  def build(self, enabled, turn, distance, v_ego=11.0, experimental=True):
    from openpilot.cereal import messaging
    from openpilot.sunnypilot.selfdrive.controls.lib.dec.tests.test_dec_planner_gate import build_planner, build_sm
    from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_fm_nav_turn_speed import FakeParams as P
    sm = build_sm(experimental)
    cs = messaging.new_message('carState')
    cs.carState.vEgo, cs.carState.vCruise, cs.carState.vCruiseCluster = v_ego, 72.0, 72.0
    sm['carState'] = cs.carState.as_reader()
    md = messaging.new_message('modelDataV2SP')
    md.modelDataV2SP.laneTurnDirection = turn
    sm['modelDataV2SP'] = md.modelDataV2SP.as_reader()
    n = messaging.new_message('fmNavTurn')
    n.fmNavTurn.status = 'ok'
    n.fmNavTurn.right.valid, n.fmNavTurn.right.distance = True, distance
    sm['fmNavTurn'] = n.fmNavTurn.as_reader()
    sm.logMonoTime.update(modelV2=int(10e9), fmNavTurn=int(10e9), modelDataV2SP=int(10e9))
    sm.seen = {'fmNavTurn': True, 'modelDataV2SP': True}
    planner = build_planner(False, 'acc')
    planner.fm_nav_turn_speed.params = P(enabled=enabled)
    planner.fm_nav_turn_speed.read_params()
    planner.fm_nav_turn_speed.params_t = 1e18  # keep the fake params
    return planner, sm

  def test_caps_the_cruise_target_toward_a_mapped_street(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanSource as Src
    off, sm_off = self.build(False, 'turnRight', 40.0)
    on, sm_on = self.build(True, 'turnRight', 40.0)
    off.update(sm_off)
    on.update(sm_on)
    assert on.source == Src.fmNavTurn and off.source == Src.cruise
    assert on.output_v_target < 11.0 < off.output_v_target
    assert on.a_cruise < off.a_cruise

  def test_no_turn_is_as_before(self):
    a, sm_a = self.build(True, 'none', 40.0)
    b, sm_b = self.build(False, 'none', 40.0)
    a.update(sm_a)
    b.update(sm_b)
    assert (a.source, a.output_v_target, a.a_cruise, a.output_a_target) == (b.source, b.output_v_target, b.a_cruise, b.output_a_target)

  def test_without_the_map_messages_is_as_before(self):
    a, sm_a = self.build(True, 'turnRight', 40.0)
    sm_a.seen = {}
    b, sm_b = self.build(False, 'turnRight', 40.0)
    a.update(sm_a)
    b.update(sm_b)
    assert (a.source, a.output_v_target, a.a_cruise) == (b.source, b.output_v_target, b.a_cruise)
