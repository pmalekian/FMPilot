"""FM Pilot: Settle After Turn with Finish Turn After Signal Cancels (fm-dev-sync).

Turn Assist's carry (fmControls.turnAssist, source carry) keeps a turn going after its desire ends at turn-in. Settle
treats the carry as the turn still on and the end of the carry as the end of the turn: with a carry, Settle gives
exactly what it gives for the same drive with the desire kept on through the carry; with no carry, the carry input
changes nothing.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import custom, log
from openpilot.sunnypilot.modeld_v2 import fm_turn_settle as fts
from openpilot.sunnypilot.modeld_v2.tests.test_fm_turn_settle import output

DT = fts.DT
STATE = ("sign", "turned", "time_left", "curvature", "offset", "active", "push", "limit", "phase", "guard", "settled", "peak",
         "straight_time", "prev_desire_sign")


def state(s):
  return tuple(getattr(s, f) for f in STATE)


def inputs(out=None, v=6.7, k=0.0):
  """TurnSettle.update's model inputs, taken from a model output the way settle_from_model takes them."""
  from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan
  out = output(center=0.6, probs=(0.35, 0.35)) if out is None else out
  plan = out['plan'][0, :, Plan.POSITION]
  x = np.asarray(ModelConstants.X_IDXS)
  return {'lane_changing': False, 'v_ego': v, 'truck_curvature': k, 'lat_active': True, 'steering_pressed': False,
          'blendful_trust': 0.0, 'x_idxs': x, 'lines_y': out['lane_lines'][0, :, :, 0],
          'line_probs': out['lane_lines_prob'][0, 1::2], 'line_stds': out['lane_lines_stds'][0, :, 0, 0],
          'edges_y': out['road_edges'][0, :, :, 0], 'edge_stds': out['road_edges_stds'][0, :, 0, 0],
          'plan_x': plan[:, 0], 'plan_y': plan[:, 1]}


def drive(sign, early, desire_frames, carry_frames, gap=0, after=140, keep_desire=False, carry_sign=None, v=4.0):
  """A turn of `desire_frames` with the desire on (truck at 0.1 1/m toward it), then `gap` frames of nothing, then
  `carry_frames` of carry (still turning), then straight. keep_desire: the same drive with the desire on instead of the
  carry (what the carry should look like). Returns the per-frame outputs and states."""
  s = fts.TurnSettle()
  carry_sign = sign if carry_sign is None else carry_sign
  res = []
  plan = [(sign, 0, sign * 0.1)] * desire_frames + [(0, 0, sign * 0.1)] * gap + \
         [(sign if keep_desire else 0, 0 if keep_desire else carry_sign, sign * 0.1)] * carry_frames + [(0, 0, 0.0)] * after
  for d, c, k in plan:
    out = s.update(True, d, early=early, model_curvature=0.0, carry_sign=c, **inputs(v=v, k=k))
    res.append((out, state(s)))
  return res


def frames_for(degrees, v=4.0):
  return int(round(math.radians(degrees) / (0.1 * v * DT)))


@pytest.mark.parametrize("early", [False, True])
@pytest.mark.parametrize("sign", [1, -1])
class TestCarryIsTheTurn:
  def test_carry_equals_desire_kept_on(self, early, sign):
    a = drive(sign, early, frames_for(30), frames_for(60), keep_desire=True)
    b = drive(sign, early, frames_for(30), frames_for(60))
    assert a == b

  def test_window_opens_after_the_carried_turn(self, early, sign):
    """30 deg with the blinker, 60 more carried: before this, the blinker cancel at 30 deg was the end of a turn
    under 45 deg, so no window ever opened."""
    res = drive(sign, early, frames_for(30), frames_for(60))
    assert any(st[STATE.index("time_left")] > 0.0 for _, st in res)
    no_carry = drive(sign, early, frames_for(30), frames_for(60), carry_sign=0)
    assert not any(st[STATE.index("time_left")] > 0.0 for _, st in no_carry)

  @pytest.mark.parametrize("gap", [1, 2, 3])
  def test_late_carry_takes_the_turn_back(self, early, sign, gap):
    """controlsd's carry shows up 1-3 model frames after modeld saw the desire end: the turn is taken back with what
    it had turned (only the gap frames aren't counted), so the window still opens after the carry."""
    res = drive(sign, early, frames_for(30), frames_for(60), gap=gap)
    i_carry = frames_for(30) + gap
    st = res[i_carry][1]
    assert st[STATE.index("sign")] == sign and st[STATE.index("turned")] > math.radians(29)
    assert any(s[STATE.index("time_left")] > 0.0 for _, s in res[i_carry + frames_for(60):])

  def test_late_carry_undoes_a_window_opened_at_the_cancel(self, early, sign):
    """Settle's 20 Hz count can reach 45 deg a little before Turn Assist's: a window opened at the cancel is undone
    when the carry shows up a frame later, and opens again once the carry ends."""
    n = frames_for(46)
    res = drive(sign, early, n, frames_for(40), gap=1)
    assert res[n][1][STATE.index("time_left")] > 0.0  # opened at the cancel
    during = res[n + 1:n + 1 + frames_for(40)]
    assert all(st[STATE.index("time_left")] == 0.0 for _, st in during)  # undone through the carry
    assert all(st[STATE.index("turned")] > math.radians(45) for _, st in during)
    assert any(st[STATE.index("time_left")] > 0.0 for _, st in res[n + 1 + frames_for(40):])

  def test_carry_too_late_or_other_side_is_ignored(self, early, sign):
    """A carry more than CARRY_GRACE frames after the turn ended, or of the other side, isn't this turn: exactly as with
    no carry at all (the cancel under 45 deg ended the turn)."""
    for kw in ({"gap": fts.CARRY_GRACE + 1}, {"carry_sign": -sign}):
      res = drive(sign, early, frames_for(30), frames_for(60), **kw)
      none = drive(sign, early, frames_for(30), frames_for(60), **(kw | {"carry_sign": 0}))
      assert res == none
      assert not any(st[STATE.index("time_left")] > 0.0 for _, st in res)


@pytest.mark.parametrize("early", [False, True])
def test_no_carry_input_changes_nothing(early):
  """Random drives: carry_sign 0 everywhere gives the same outputs and states as not passing it at all."""
  rng = np.random.default_rng(7)
  a, b = fts.TurnSettle(), fts.TurnSettle()
  desire = 0
  for _ in range(4000):
    if rng.random() < 0.03:
      desire = int(rng.choice([0, 0, 1, -1]))
    kw = inputs(output(center=float(rng.normal(0, 0.8)), probs=tuple(rng.uniform(0, 0.7, 2))),
                v=float(rng.uniform(0, 15)), k=float(rng.normal(0, 0.06)))
    kw['steering_pressed'] = bool(rng.random() < 0.01)
    x = a.update(True, desire, early=early, **kw)
    y = b.update(True, desire, early=early, carry_sign=0, **kw)
    assert x == y and state(a) == state(b)


class FakeSM(dict):
  def __init__(self, k=0.0, ta=None):
    super().__init__(controlsState=SimpleNamespace(curvature=k), carControl=SimpleNamespace(latActive=True),
                     carState=SimpleNamespace(steeringPressed=False))
    self.seen = {'fmControls': ta is not None}
    if ta is not None:
      self['fmControls'] = SimpleNamespace(turnAssist=ta)


def test_carry_read_from_fm_controls():
  ta = custom.FmControls.new_message().turnAssist
  assert fts.carry_from_sm(FakeSM()) == 0
  assert fts.carry_from_sm({}) == 0
  ta.sign, ta.source = -1, 'carry'
  assert fts.carry_from_sm(FakeSM(ta=ta)) == -1
  ta.source = 'desire'
  assert fts.carry_from_sm(FakeSM(ta=ta)) == 0


def test_settle_from_model_follows_the_carry():
  """modeld_v2's call: the carry from fmControls keeps the turn going exactly as the desire would."""
  ta = custom.FmControls.new_message().turnAssist
  ta.sign, ta.source = 1, 'carry'
  out = output(center=0.6)
  a, b = fts.TurnSettle(), fts.TurnSettle()
  n, m = frames_for(30, v=6.7), frames_for(60, v=6.7)
  opened = False
  for i in range(n + m + 100):
    k = 0.1 if i < n + m else 0.0
    da = log.Desire.turnRight if i < n + m else log.Desire.none
    db = log.Desire.turnRight if i < n else log.Desire.none
    sm_b = FakeSM(k, ta if n <= i < n + m else None)
    x = fts.settle_from_model(a, True, da, log.LaneChangeState.off, 6.7, FakeSM(k), 0.0, out)
    y = fts.settle_from_model(b, True, db, log.LaneChangeState.off, 6.7, sm_b, 0.0, out)
    assert x == y and state(a) == state(b)
    opened = opened or b.time_left > 0.0
  assert opened


def test_modeld_v2_wiring():
  """modeld_v2 subscribes to fmControls (not polled: ignore_alive/avg_freq/valid), runs Settle in the Blendful hook and
  logs it in fmModel."""
  from pathlib import Path
  src = (Path(fts.__file__).parent / "modeld.py").read_text()
  assert '"fmControls"' in src and 'ignore_alive=["radarState", "fmControls"]' in src
  assert "settle_from_model(settle, turn_settle, desire, DH.lane_change_state, v_ego, sm," in src
  for field in ("turnSettle", "turnSettleOffset", "turnSettlePush", "turnSettleLimit", "turnSettlePhase", "turnSettleGuard"):
    assert f"fm_send.fmModel.{field} = " in src
  assert set(custom.FmModel.schema.fieldnames) >= {"queueGuard", "turnSettle", "turnSettleOffset", "turnSettlePush",
                                                    "turnSettleLimit", "turnSettlePhase", "turnSettleGuard"}
