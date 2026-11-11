from types import SimpleNamespace

import numpy as np

from openpilot.cereal import log, custom
from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_pulse import FmTurnPulse, PARAM, PULL_AWAY_SPEED

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
