"""FM Pilot: Queue Guard (fm_queue_guard.py) and its wiring in DesireHelper and Turn Assist.

Ported from fm-dev-nav-settle ca08aac9 (test_fm_queue_guard.py), plus: off by default, the DesireHelper call modeld
makes, and Turn Assist never taking a hold for the blinker cancelling mid-turn (no carry from a hold).
"""
import math
import re
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import log, custom
from openpilot.common.realtime import DT_MDL
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.fm_queue_guard import FmQueueGuard, PARAM, LEAD_CLOSE, LEAD_CLEAR, QUEUE_SPEED
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LANE_CHANGE_SPEED_MIN

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
    dh.lane_turn_controller.lane_turn_value = LANE_CHANGE_SPEED_MIN
    dh.lane_turn_controller.read_params = lambda: None
    dh.fm_queue_guard.params = FakeParams(enabled=enabled)
    dh.fm_queue_guard.read_params()
    return dh

  @staticmethod
  def lead(d):
    return SimpleNamespace(present=d is not None, dRel=d if d is not None else 0.0)

  def test_model_refresh_preserves_published_turn(self):
    dh = self.make(False)
    dh.turn_repulse = 3
    for expected in (log.Desire.turnLeft, log.Desire.turnLeft, log.Desire.none, log.Desire.turnLeft):
      dh.update(cs(2.0, left=True), False, 0.0)
      assert dh.desire == log.Desire.turnLeft
      assert dh.model_desire == expected

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


REPO = Path(__file__).resolve().parents[6]


class TestDefaultOff:
  def test_param_declared_off(self):
    keys = (REPO / "openpilot" / "common" / "params_keys.h").read_text()
    assert re.search(r'\{"FM_TurnQueueGuard", \{PERSISTENT \| BACKUP, BOOL, "0"\}\}', keys)

  def test_unset_param_reads_off(self):
    g = FmQueueGuard(FakeParams(None))
    assert not g.enabled
    assert not step(g, 0.0, 3.0, n=60)

  def test_switched_on_while_driving_is_read_on_the_50th_frame(self):
    params = FakeParams(False)
    g = FmQueueGuard(params)
    params.values[PARAM] = True
    assert not step(g, 0.0, 3.0, n=49)
    assert step(g, 0.0, 3.0)  # the setting is read every 50 model frames (2.5 s)


def carry_assist():
  from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import FmTurnAssist, PARAM_HOLD, PARAM_CARRY

  class P(FakeParams):
    def __init__(self):
      self.values = {PARAM_HOLD: True, PARAM_CARRY: True, "FM_TurnPathMinRadius": 12.0, "FM_TurnLeadMinSpeed": 7.0,
                     "FM_TurnLeadGateDist": 5.0}
  return FmTurnAssist(P())


def ta_run(a, k, direction, mph, steps=1, actual=None, lat=True, queue_hold=False):
  out = k
  for _ in range(steps):
    out = a.update(k, direction, mph * CV.MPH_TO_MS, k if actual is None else actual, False, lat, queue_hold=queue_hold)
  return out


class TestNoCarryFromAHold:
  """A Queue Guard hold hides the turn desire, which Turn Assist would otherwise read as the blinker cancelling at
  turn-in (Finish Turn After Signal Cancels). ca08aac9 relied on the hold only starting stopped (lateral off there, so
  any carry ends at once); here the hold itself (fmModel.queueGuard) is never a carry, even on a step where lateral is
  still on and the truck is turning toward the turn (the hidden desire can reach controlsd a step after the stop)."""

  @pytest.mark.parametrize("direction, sign", [(TurnDirection.turnRight, 1), (TurnDirection.turnLeft, -1)])
  def test_hold_with_lateral_on_and_truck_turning_does_not_carry(self, direction, sign):
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import SOURCE_CARRY
    a = carry_assist()
    ta_run(a, sign * 0.09, direction, mph=1, steps=60, actual=sign * 0.001)
    out = ta_run(a, sign * 0.01, TurnDirection.none, mph=1, steps=1, actual=sign * 0.08, queue_hold=True)
    assert a.carry_sign == 0 and a.carry_done and a.log_source != SOURCE_CARRY
    # and stays that way through a creep behind the car (lateral on, the truck still curving toward the turn)
    for _ in range(300):
      out = ta_run(a, sign * 0.01, TurnDirection.none, mph=3, actual=sign * 0.06, queue_hold=True)
      assert a.carry_sign == 0 and a.log_source != SOURCE_CARRY
    assert out == pytest.approx(sign * 0.01)  # the hold's cancel fade is long over: the model's request alone

  def test_same_inputs_without_the_hold_do_carry(self):
    """Control case: the same blinker cancel, not a hold, is a carry (Finish Turn After Signal Cancels)."""
    a = carry_assist()
    ta_run(a, 0.09, TurnDirection.turnRight, mph=1, steps=60, actual=0.001)
    ta_run(a, 0.01, TurnDirection.none, mph=1, steps=1, actual=0.08)
    assert a.carry_sign == 1

  def test_turn_comes_back_fresh_after_the_hold(self):
    a = carry_assist()
    ta_run(a, 0.09, TurnDirection.turnRight, mph=1, steps=60, actual=0.001)
    ta_run(a, 0.01, TurnDirection.none, mph=0, steps=40, actual=0.0, lat=False, queue_hold=True)
    assert a.sign == 0 and a.peak == 0.0
    ta_run(a, 0.02, TurnDirection.turnRight, mph=2, steps=1, actual=0.02)
    assert a.sign == 1 and not a.carry_done  # a new turn: a later blinker cancel can carry again
    ta_run(a, 0.09, TurnDirection.turnRight, mph=2, steps=60, actual=0.02)
    ta_run(a, 0.01, TurnDirection.none, mph=2, steps=1, actual=0.08)
    assert a.carry_sign == 1

  def test_closed_loop_queue_never_carries(self):
    """DesireHelper (Queue Guard on) feeding Turn Assist (Turn Hold and the carry on) through a queue at a stop sign:
    lateral is on above 0.3 m/s as in controlsd, the hidden desire and the hold reach Turn Assist together, and the
    truck creeps with the wheel turned toward the turn. No step is ever a carry; the turn comes back when the car ahead
    pulls away."""
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import SOURCE_CARRY
    dh = TestDesireHelperQueueGuard().make(True)
    a = carry_assist()
    lead = TestDesireHelperQueueGuard.lead
    # (speed m/s, lead m, truck curvature) at 20 Hz; each model frame is 5 control steps
    plan = [(2.0, 9.0, 0.0)] * 10 + [(0.5, 7.5, -0.02)] * 4 + [(0.29, 7.0, -0.03)] + [(0.35, 6.5, -0.05)] * 3 + \
           [(0.0, 3.0, -0.05)] * 40 + [(1.5, 5.0, -0.06)] * 20 + [(0.0, 4.0, -0.06)] * 20 + [(1.5, 12.0, -0.08)] * 20
    held = carried = 0
    for v, d, k in plan:
      dh.update(cs(v, left=True), v > 0.3, 0.0, lead=lead(d), curvature=k)
      held += dh.fm_queue_hold
      for _ in range(5):
        a.update(k, dh.lane_turn_direction, v, k, False, v > 0.3, queue_hold=dh.fm_queue_hold)
        carried += a.log_source == SOURCE_CARRY
    assert held > 0 and carried == 0
    assert dh.lane_turn_direction == TurnDirection.turnLeft and a.sign == -1  # the car pulled away: a fresh turn

  def test_fm_lateral_reads_the_hold_from_controlsds_submaster(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_lateral import FmLateral
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import PARAM_HOLD, PARAM_CARRY

    class P(FakeParams):
      def __init__(self):
        self.values = {PARAM_HOLD: True, PARAM_CARRY: True, "FM_TurnPathMinRadius": 12.0, "FM_TurnLeadMinSpeed": 7.0,
                       "FM_TurnLeadGateDist": 5.0, "FM_LaneKeepMode": 0, "IsRhdDetected": False}

    class SM(dict):
      def __init__(self, direction, hold, fm_model_seen=True):
        super().__init__(modelDataV2SP=SimpleNamespace(laneTurnDirection=direction), fmModel=SimpleNamespace(queueGuard=hold))
        self.seen = {'modelDataV2SP': True, 'fmModel': fm_model_seen}

    CS = SimpleNamespace(vEgo=1.0, steeringPressed=False, leftBlinker=False, rightBlinker=True, steeringTorque=0.0, aEgo=0.0)
    for hold, seen, carries in ((True, True, False), (False, True, True), (True, False, True)):
      lat = FmLateral(SimpleNamespace(brand='rivian'), P())
      lat.lane_guard.update = lambda *a, **k: 0.0
      for _ in range(60):
        lat.update(0.09, SimpleNamespace(), CS, True, False, 0.001, SM(TurnDirection.turnRight, False))
      lat.update(0.01, SimpleNamespace(), CS, True, False, 0.08, SM(TurnDirection.none, hold, seen))
      assert (lat.turn_assist.carry_sign == 1) == carries


class TestModeldWiring:
  """Both modelds pass radarState's lead and controlsState's curvature to DesireHelper, and send fmModel (with the
  hold) before modelDataV2SP (with the hidden desire)."""

  @pytest.mark.parametrize("path", ["openpilot/selfdrive/modeld/modeld.py", "openpilot/sunnypilot/modeld_v2/modeld.py"])
  def test_wiring(self, path):
    src = (REPO / path).read_text()
    assert '"radarState"' in src and '"fmModel"' in src
    assert "lead=queue_lead, curvature=sm['controlsState'].curvature" in src
    assert "fm_send.fmModel.queueGuard = bool(DH.fm_queue_hold)" in src
    assert src.index("pm.send('fmModel', fm_send)") < src.index("pm.send('modelDataV2SP', mdv2sp_send)")

  def test_controlsd_subscribes_to_fm_model(self):
    src = (REPO / "openpilot/selfdrive/controls/controlsd.py").read_text()
    assert "'fmModel'" in src


PRE_POST_KEYS = ("FM_TurnQueueGuard", "FM_TurnPulseOnPullAway", "FM_TurnSettle", "FM_TurnSettleEarlyPush")


class TestPrePostTurnPage:
  """Steering > Turns > Pre/Post Turn Behavior: a plain sub-page (always openable: its trigger is its own first item,
  with no trigger condition, as sunnypilot's Speed Limit Settings), the four toggles in order, all off on a fresh
  install."""

  def _turns(self):
    import yaml
    page = yaml.safe_load((REPO / "openpilot/sunnypilot/sunnylink/settings_ui_src/pages/steering.yaml").read_text())
    return {s["id"]: s for s in page["sections"]}["fm_turns"]

  def test_sub_page(self):
    turns = self._turns()
    (sub,) = [s for s in turns["sub_panels"] if s["id"] == "pre_post_turn"]
    assert sub["label"] == "Pre/Post Turn Behavior" and sub["trigger_key"] == "FM_TurnQueueGuard"
    assert "trigger_condition" not in sub
    items = {i["key"]: i for i in sub["items"]}
    assert tuple(items) == PRE_POST_KEYS
    assert [items[k]["title"] for k in PRE_POST_KEYS] == ["Queue Guard", "Turn Pulse on Pull-Away", "Settle After Turn",
                                                          "Settle After Turn: Early Right Push"]
    assert all(i["widget"] == "toggle" for i in items.values())
    assert items["FM_TurnSettleEarlyPush"]["enablement"] == [{"type": "param", "key": "FM_TurnSettle", "equals": True}]
    assert all("enablement" not in items[k] for k in PRE_POST_KEYS[:3])
    assert "10 mph" in items["FM_TurnQueueGuard"]["description"]  # QUEUE_SPEED (ca08aac9's text said 5 mph)
    assert not any(k in i["key"] for i in turns["items"] for k in PRE_POST_KEYS)

  def test_compiled_json_has_it(self):
    import json
    data = json.loads((REPO / "openpilot/sunnypilot/sunnylink/settings_ui.json").read_text())
    found = [sp for panel in data["panels"] for sec in panel.get("sections", []) for sp in sec.get("sub_panels", [])
             if sp["id"] == "pre_post_turn"]
    assert len(found) == 1 and [i["key"] for i in found[0]["items"]] == list(PRE_POST_KEYS)

  def test_defaults_off(self):
    from openpilot.common.params import Params
    params = Params()
    for key in PRE_POST_KEYS:
      assert params.get_default_value(key) is False, key
