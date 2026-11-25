"""FM Pilot: Turn Pulse on Pull-Away (fm_turn_pulse.py) and its wiring in DesireHelper and both modelds.

Ported from fm-dev-nav-settle ca08aac9 (test_fm_turn_pulse.py), plus: off by default, the end of the turn (the desire,
or a Finish Turn After Signal Cancels carry after it) clears the stop, and the modelds feed DH.model_desire.
"""
import re
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log, custom
from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_pulse import FmTurnPulse, PARAM, PULL_AWAY_SPEED
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LANE_CHANGE_SPEED_MIN

TurnDirection = custom.ModelDataV2SP.TurnDirection
TURN = log.Desire.turnRight


class FakeParams:
  def __init__(self, enabled=True):
    self.values = {PARAM: enabled}

  def get_bool(self, key):
    return bool(self.values.get(key))

  def get(self, key, return_default=False):
    return self.values.get(key)


def model_pulses(desires):
  """modeld's rule: the model only gets a desire on the frame it rises."""
  prev, out = np.zeros(8), []
  for d in desires:
    vec = np.zeros(8)
    vec[d] = 1
    out.append(int(np.argmax(np.where(vec - prev > .99, vec, 0))) if (vec - prev > .99).any() else 0)
    prev = vec
  return out


class TestFmTurnPulse:
  def test_off_passes_through(self):
    p = FmTurnPulse(FakeParams(enabled=False))
    assert [p.update(TURN, v) for v in (2.0, 0.0, 0.0, 1.0)] == [TURN] * 4

  def test_pull_away_drops_one_frame(self):
    p = FmTurnPulse(FakeParams())
    speeds = [2.0, 1.0, 0.0, 0.0, 0.0, 0.2, PULL_AWAY_SPEED, 0.5, 1.0]
    out = [p.update(TURN, v) for v in speeds]
    assert out == [TURN] * 6 + [log.Desire.none] + [TURN] * 2

  def test_model_gets_a_fresh_pulse(self):
    p = FmTurnPulse(FakeParams())
    speeds = [0.0] * 5 + [1.0] * 5
    pulses = model_pulses([p.update(TURN, v) for v in speeds])
    assert pulses.count(TURN) == 2  # when the turn starts, and when pulling away

  def test_no_pulse_without_a_stop(self):
    p = FmTurnPulse(FakeParams())
    assert log.Desire.none not in [p.update(TURN, v) for v in (5.0, 3.0, 1.0, 0.5, 2.0)]

  def test_once_per_stop(self):
    p = FmTurnPulse(FakeParams())
    out = [p.update(TURN, v) for v in (0.0, 1.0, 1.0, 0.2, 1.0)]  # slows to 0.2 m/s, not a stop
    assert out.count(log.Desire.none) == 1

  def test_each_stop_pulses_again(self):
    p = FmTurnPulse(FakeParams())
    out = [p.update(TURN, v) for v in (0.0, 1.0, 0.0, 1.0)]
    assert out.count(log.Desire.none) == 2

  def test_stop_before_turn_doesnt_count(self):
    p = FmTurnPulse(FakeParams())
    p.update(log.Desire.none, 0.0)
    assert p.update(TURN, 1.0) == TURN

  def test_turn_ended_clears_the_stop(self):
    p = FmTurnPulse(FakeParams())
    p.update(TURN, 0.0)
    p.update(log.Desire.none, 0.0)
    assert p.update(TURN, 1.0) == TURN

  def test_lane_changes_untouched(self):
    p = FmTurnPulse(FakeParams())
    out = [p.update(log.Desire.laneChangeLeft, v) for v in (0.0, 1.0)]
    assert out == [log.Desire.laneChangeLeft] * 2


def cs(v, right=False):
  return SimpleNamespace(vEgo=v, leftBlinker=False, rightBlinker=right, steeringPressed=False, steeringTorque=0.0,
                         leftBlindspot=False, rightBlindspot=False, brakePressed=False)


class TestDesireHelperIntegration:
  def make(self, enabled):
    from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
    dh = DesireHelper()
    dh.lane_turn_controller.enabled = True
    dh.lane_turn_controller.lane_turn_value = LANE_CHANGE_SPEED_MIN
    dh.lane_turn_controller.read_params = lambda: None
    dh.fm_turn_pulse.params = FakeParams(enabled=enabled)
    dh.fm_turn_pulse.read_params()
    return dh

  def test_turn_kept_model_input_blips(self):
    dh = self.make(True)
    for v in (0.0, 0.0):
      dh.update(cs(v, right=True), False, 0.0)
    dh.update(cs(1.0, right=True), True, 0.0)
    assert dh.desire == TURN
    assert dh.lane_turn_direction == TurnDirection.turnRight
    assert dh.model_desire == log.Desire.none
    dh.update(cs(1.2, right=True), True, 0.0)
    assert dh.model_desire == TURN

  def test_off_model_input_is_desire(self):
    dh = self.make(False)
    for v in (0.0, 1.0, 1.2):
      dh.update(cs(v, right=True), v > 0.3, 0.0)
      assert dh.model_desire == dh.desire == TURN

  def test_turn_ended_at_the_stop_no_pulse_on_pull_away(self):
    """The blinker goes off while stopped (or a carry after it ends: Turn Assist's carry ends with lateral off, under
    0.3 m/s, and never feeds the model): the turn is over, so pulling away gives the model nothing."""
    dh = self.make(True)
    for v in (1.0, 0.0, 0.0):
      dh.update(cs(v, right=True), v > 0.3, 0.0)
    dh.update(cs(0.0), False, 0.0)
    out = []
    for v in (0.2, PULL_AWAY_SPEED, 1.0, 2.0):
      dh.update(cs(v), v > 0.3, 0.0)
      out.append(dh.model_desire)
    assert out == [log.Desire.none] * 4
    assert not dh.fm_turn_pulse.stopped

  def test_queue_guard_hold_then_release_is_one_fresh_turn(self):
    """With Queue Guard holding, the desire (and so the model's input) is none; when it lets go the turn comes back as
    a rising edge on its own, and Turn Pulse doesn't add a second blip at the pull-away."""
    dh = self.make(True)
    dh.fm_queue_guard.params = FakeParams(enabled=True)
    dh.fm_queue_guard.params.values["FM_TurnQueueGuard"] = True
    dh.fm_queue_guard.read_params()
    lead = SimpleNamespace(present=True, dRel=3.0)
    seq = []
    for v, d in [(0.0, 3.0)] * 5 + [(1.0, 12.0)] * 5:
      lead.dRel = d
      dh.update(cs(v, right=True), v > 0.3, 0.0, lead=lead)
      seq.append(dh.model_desire)
    assert seq == [log.Desire.none] * 5 + [TURN] * 5
    assert model_pulses(seq).count(TURN) == 1


REPO = Path(__file__).resolve().parents[6]


class TestDefaultsAndWiring:
  def test_param_declared_off(self):
    keys = (REPO / "openpilot" / "common" / "params_keys.h").read_text()
    assert re.search(r'\{"FM_TurnPulseOnPullAway", \{PERSISTENT \| BACKUP, BOOL, "0"\}\}', keys)

  def test_unset_param_reads_off(self):
    p = FmTurnPulse(FakeParams(enabled=None))
    assert not p.enabled
    assert [p.update(TURN, v) for v in (0.0, 1.0)] == [TURN, TURN]

  @pytest.mark.parametrize("path", ["openpilot/selfdrive/modeld/modeld.py", "openpilot/sunnypilot/modeld_v2/modeld.py"])
  def test_modelds_feed_the_model_desire(self, path):
    src = (REPO / path).read_text()
    assert "model_desire = DH.model_desire" in src
    assert "vec_desire[model_desire] = 1" in src and "vec_desire[desire] = 1" not in src
