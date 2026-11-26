"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

FM Pilot: sunnypilot's Blendful tests from #2073 (unchanged), plus FM Pilot's checks at the bottom.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.modeld_v2.blendful import MIN_SPEED, Blendful
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants

X = np.array(ModelConstants.X_IDXS)
BUILD, RELEASE = 0.5, 2.0


def frame(center=0.0, plan=0.0, k=0.0, probs=(0.9, 0.9), stds=(0.1, 0.1), width=3.5):
  road = k * X ** 2 / 2
  lines = np.zeros((1, 4, 33, 2))
  lane_stds = np.ones((1, 4, 33, 2))
  for i, off in enumerate((-1.5 * width, -width / 2, width / 2, 1.5 * width)):
    lines[0, i, :, 0] = road + center + off
  lane_stds[0, 1, :, 0], lane_stds[0, 2, :, 0] = stds
  lane_probs = np.zeros((1, 8))
  lane_probs[0, 3], lane_probs[0, 5] = probs
  path = np.zeros((1, 33, 3))
  path[0, :, 0] = X
  path[0, :, 1] = road + plan * np.clip(X / 10, 0, 1)
  return {'lane_lines': lines, 'lane_lines_stds': lane_stds, 'lane_lines_prob': lane_probs, 'plan': path}


def drive(bf, seconds, v=20.0, k=0.0, **kw):
  out = [bf.update(frame(k=k, **kw), v, True, k) for _ in range(int(seconds / DT_MDL))]
  return np.array(out) * v ** 2  # m/s^2


class TestBlendful(OpenpilotTestCase):
  def setup_method(self):
    self.bf = Blendful()

  def test_centered_adds_nothing(self):
    push = drive(self.bf, 5.0)
    assert self.bf.trust > 0.9
    assert np.abs(push).max() < 1e-3

  def test_pulls_toward_lane_center(self):
    push = drive(self.bf, 5.0, center=0.5)
    assert push[-1] > 0.02  # + y side, where the lane center is
    assert self.bf.trust > 0.9
    assert np.all(np.diff(push) <= BUILD * DT_MDL + 1e-9)

  def test_flipping_sides_builds_slowly(self):
    k = 0.002  # 0.8 m/s^2 at 20 m/s
    drive(self.bf, 5.0, k=k, center=0.5)
    push = drive(self.bf, 5.0, k=k, center=-0.5)
    rate = np.diff(push) / DT_MDL
    assert push[-1] < 0
    assert np.all(rate >= -RELEASE - 1e-6)
    assert np.all(rate[push[:-1] <= 0] >= -BUILD - 1e-6)  # once past zero it builds
    crossing = push[1:][(push[:-1] > 0) & (push[1:] <= 0)]
    assert len(crossing) and np.all(crossing >= -BUILD * DT_MDL - 1e-9)

  def test_weak_lines_drive_laneless(self):
    clear = drive(Blendful(), 5.0, center=0.5)[-1]
    faint = Blendful()
    push = drive(faint, 5.0, center=0.5, probs=(0.1, 0.1))
    assert faint.trust < 0.25 and abs(push[-1]) < 0.25 * clear  # faint lines = trust and push scale down
    blurry = Blendful()
    push = drive(blurry, 5.0, center=0.5, stds=(0.8, 0.8))
    assert blurry.trust == 0.0 and np.abs(push).max() < 1e-6  # lines too uncertain = laneless

  def test_lone_line_holds_through_a_blink_in_a_curve(self):
    k, v = 1 / 20, 6.0
    drive(self.bf, 6.0, v=v, k=k, plan=0.4, probs=(0.9, 0.05))
    before = self.bf.trust
    drive(self.bf, 0.25, v=v, k=k, plan=0.4, probs=(0.1, 0.05), stds=(1.0, 0.1))
    bottom = self.bf.trust
    drive(self.bf, 2.0, v=v, k=k, plan=0.4, probs=(0.9, 0.05))
    assert before > 0.5
    assert bottom > 0.5 * before  # keeps latched through flicker
    assert self.bf.trust > 0.85 * before

  def test_yields_when_the_model_goes_around(self):
    drive(self.bf, 6.0)
    push = drive(self.bf, 3.0, plan=0.8)
    assert self.bf.going_around
    assert self.bf.trust < 0.05
    assert abs(push[-1]) < 0.01

  def test_fit_does_not_grow_while_trust_falls(self):
    drive(self.bf, 5.0, center=0.3)
    for i in range(40):
      dk, trust = self.bf.dk, self.bf.trust
      p = 0.9 - 0.02 * i
      self.bf.update(frame(center=0.3, plan=-0.02 * i, probs=(p, p)), 20.0, True, 0.0)
      if self.bf.trust < trust:
        assert abs(self.bf.dk) <= abs(dk) + 1e-12

  def test_toggle_off_returns_zero_and_resets(self):
    drive(self.bf, 5.0, center=0.5)
    assert self.bf.curvature != 0.0
    assert self.bf.update(frame(center=0.5), 20.0, True, 0.0, enabled=False) == 0.0
    assert self.bf.curvature == 0.0 and self.bf.trust == 0.0

  def test_fades_below_min_speed(self):
    drive(self.bf, 5.0, v=10.0, center=0.5)
    start = self.bf.curvature
    out = np.array([self.bf.update(frame(center=0.5), 0.5 * MIN_SPEED, True, 0.0) for _ in range(100)])  # FM: was 4.0 (MIN_SPEED 5.0)
    assert self.bf.trust == 0.0
    assert abs(out[0]) < abs(start)
    assert abs(out[-1]) < 1e-6

  # FM Pilot

  def test_off_is_exactly_zero_on_every_frame(self):
    # Not selected = the model's steering request is untouched, bit for bit
    rng = np.random.default_rng(0)
    for _ in range(400):
      out = self.bf.update(frame(center=rng.uniform(-1, 1), plan=rng.uniform(-1, 1), k=rng.uniform(-0.01, 0.01)),
                           rng.uniform(0, 35), True, rng.uniform(-0.01, 0.01), enabled=False)
      assert out == 0.0
      assert self.bf.curvature == 0.0 and self.bf.trust == 0.0

  def test_nothing_during_a_turn_or_lane_change(self):
    # modeld passes active=False while a desire (turn or lane change) is on
    drive(self.bf, 5.0, center=0.5)
    out = [self.bf.update(frame(center=0.5), 20.0, False, 0.0) for _ in range(200)]
    assert self.bf.trust == 0.0
    assert abs(out[-1]) < 1e-9

  def test_works_in_slow_traffic(self):
    # FM Pilot: MIN_SPEED 0.3 m/s (was 5.0), so Blendful centers in stop-and-go traffic too
    assert MIN_SPEED == 0.3
    for v in (1.0, 2.0, 4.0):  # about 2, 4.5 and 9 mph
      bf = Blendful()
      push = drive(bf, 5.0, v=v, center=0.5)
      assert bf.trust > 0.9
      assert push[-1] > 0.0 and bf.curvature > 0.0  # toward the lane center, as at 20 m/s

  def test_stopping_resets_and_pulling_away_rebuilds(self):
    drive(self.bf, 5.0, v=2.0, center=0.5)
    start = self.bf.curvature
    out = np.array([self.bf.update(frame(center=0.5), 0.0, True, 0.0) for _ in range(40)])  # stopped 2 s
    assert self.bf.trust == 0.0
    assert np.all(np.diff(np.abs(out)) <= 1e-12) and abs(out[-1]) < 0.05 * abs(start)  # fades out smoothly (0.5 s)
    self.bf.update(frame(center=0.5), 1.0, True, 0.0)
    assert self.bf.trust < 0.1  # trust ramps back up (1.5 s), no jump on pull-away
    drive(self.bf, 5.0, v=1.0, center=0.5)
    assert self.bf.trust > 0.9

  def test_waits_for_turn_carry_on_small_and_jetlink_paths(self):
    # FM Pilot: while Finish Turn After Signal Cancels carries a turn (fmControls.turnAssist.source == carry),
    # Blendful adds nothing on either the small-model path or the Jetlink big-model path, then resumes
    # bit-for-bit like after a normal blinker turn.
    from functools import partial
    from types import SimpleNamespace

    from openpilot.cereal import custom, log
    from openpilot.selfdrive.modeld import modeld as stock_modeld
    from openpilot.sunnypilot.modeld_v2.fm_jetlink_action import with_blendful
    from openpilot.sunnypilot.modeld_v2.modeld import ModelState, is_blendful_active

    class FakeSM(dict):
      def __init__(self, source='none', sign=0, seen=True):
        ta = custom.FmControls.new_message().turnAssist
        ta.source = source
        ta.sign = sign
        super().__init__(fmControls=SimpleNamespace(turnAssist=ta))
        self.seen = {'fmControls': seen}

    small = ModelState.__new__(ModelState)
    small.generation = None
    small.LONG_SMOOTH_SECONDS = stock_modeld.LONG_SMOOTH_SECONDS
    small.MIN_LAT_CONTROL_SPEED = stock_modeld.MIN_LAT_CONTROL_SPEED
    jetlink_big = with_blendful(stock_modeld.get_action_from_model, stock_modeld.MIN_LAT_CONTROL_SPEED)
    runners = [small.get_action_from_model, jetlink_big]

    model_output = frame(center=0.5) | {'action': np.array([[0.0, 0.0]], dtype=np.float32)}
    v_ego = 5.0  # ~11 mph: inside both Turn Assist carry and Blendful speed ranges
    lc_off = log.LaneChangeState.off

    def run_sequence(action_fn, steps):
      bf = Blendful()
      prev = log.ModelDataV2.Action()
      curvs, trusts = [], []
      for desire, sm in steps:
        active = is_blendful_active(desire, lc_off, sm)
        carry = modeld_v2_carry(sm) != 0
        lane_keep = partial(bf.update, model_output, v_ego, active, enabled=True, carry=carry)
        prev = action_fn(model_output, prev, 0.0, 0.0, v_ego, lane_keep)
        curvs.append(prev.desiredCurvature)
        trusts.append(bf.trust)
      return curvs, trusts

    from openpilot.sunnypilot.modeld_v2.fm_turn_settle import carry_from_sm as modeld_v2_carry
    no_carry = FakeSM('none', 0)
    for runner in runners:
      for sign, turn_desire in ((-1, log.Desire.turnLeft), (1, log.Desire.turnRight)):
        carry_sm = FakeSM('carry', sign)
        carry_seq = [(turn_desire, no_carry)] * 30 + [(log.Desire.none, carry_sm)] * 40 + [(log.Desire.none, no_carry)] * 60
        blinker_seq = [(turn_desire, no_carry)] * 70 + [(log.Desire.none, no_carry)] * 60

        carry_curvs, carry_trusts = run_sequence(runner, carry_seq)
        blinker_curvs, blinker_trusts = run_sequence(runner, blinker_seq)

        # During the carry, Blendful adds nothing and holds zero trust
        assert all(c == 0.0 for c in carry_curvs[30:70])
        assert all(t == 0.0 for t in carry_trusts[30:70])
        # After the carry ends, Blendful resumes identically to after a normal blinker turn
        assert carry_curvs[70:] == blinker_curvs[70:]
        assert carry_trusts[70:] == blinker_trusts[70:]
        assert carry_curvs[-1] > 0.0 and carry_trusts[-1] > 0.5

  def test_fades_already_applied_correction_during_carry_and_resumes(self):
    from functools import partial
    from openpilot.cereal import log
    from openpilot.selfdrive.modeld import modeld as stock_modeld
    from openpilot.sunnypilot.modeld_v2.blendful import CARRY_FADE_FRAMES
    from openpilot.sunnypilot.modeld_v2.fm_jetlink_action import with_blendful
    from openpilot.sunnypilot.modeld_v2.modeld import ModelState

    small = ModelState.__new__(ModelState)
    small.generation = None
    small.LONG_SMOOTH_SECONDS = stock_modeld.LONG_SMOOTH_SECONDS
    small.MIN_LAT_CONTROL_SPEED = stock_modeld.MIN_LAT_CONTROL_SPEED
    jetlink_big = with_blendful(stock_modeld.get_action_from_model, stock_modeld.MIN_LAT_CONTROL_SPEED)
    model_output = frame(center=0.5) | {'action': np.array([[0.0, 0.0]], dtype=np.float32)}
    v_ego = 5.0

    for runner in (small.get_action_from_model, jetlink_big):
      bf = Blendful()
      prev = log.ModelDataV2.Action()
      # 1. Straight driving builds a real Blendful correction
      for _ in range(100):
        prev = runner(model_output, prev, 0.0, 0.0, v_ego, partial(bf.update, model_output, v_ego, True, enabled=True))
      assert prev.desiredCurvature > 0.004
      # 2. Quick 1-frame blinker leaves a sizable residual curvature when carry begins
      prev = runner(model_output, prev, 0.0, 0.0, v_ego, partial(bf.update, model_output, v_ego, False, enabled=True))
      before_carry = prev.desiredCurvature
      assert before_carry > 0.003

      # 3. Carry starts: fades smoothly over ~0.2 s (CARRY_FADE_FRAMES) to exact 0 and stays 0 through last carry frame
      carry_curvs = []
      for _ in range(30):
        prev = runner(model_output, prev, 0.0, 0.0, v_ego,
                      partial(bf.update, model_output, v_ego, False, enabled=True, carry=True))
        carry_curvs.append(prev.desiredCurvature)
      assert 0.0 < carry_curvs[0] < before_carry
      assert all(a >= b for a, b in zip(carry_curvs[:CARRY_FADE_FRAMES], carry_curvs[1:CARRY_FADE_FRAMES], strict=False))
      assert carry_curvs[CARRY_FADE_FRAMES - 1] == 0.0
      assert all(c == 0.0 for c in carry_curvs[CARRY_FADE_FRAMES - 1:])
      assert carry_curvs[-1] == 0.0 and bf.smooth == 0.0 and bf.trust == 0.0

      # 4. Carry ends: resumes normally from zero
      resumed = []
      for _ in range(60):
        prev = runner(model_output, prev, 0.0, 0.0, v_ego,
                      partial(bf.update, model_output, v_ego, True, enabled=True, carry=False))
        resumed.append(prev.desiredCurvature)
      assert 0.0 < resumed[0] < resumed[-1] and bf.trust > 0.5

  def test_no_carry_behavior_unchanged_and_modeld_wired(self):
    from pathlib import Path
    from types import SimpleNamespace

    from openpilot.cereal import custom, log
    from openpilot.sunnypilot.modeld_v2 import modeld as modeld_v2

    class FakeSM(dict):
      def __init__(self, source='none', sign=0, seen=True):
        ta = custom.FmControls.new_message().turnAssist
        ta.source = source
        ta.sign = sign
        super().__init__(fmControls=SimpleNamespace(turnAssist=ta))
        self.seen = {'fmControls': seen}

    lc_off = log.LaneChangeState.off
    assert modeld_v2.is_blendful_active(log.Desire.none, lc_off, FakeSM(seen=False))
    assert modeld_v2.is_blendful_active(log.Desire.none, lc_off, FakeSM('none', 0))
    assert modeld_v2.is_blendful_active(log.Desire.none, lc_off, FakeSM('desire', 1))
    assert not modeld_v2.is_blendful_active(log.Desire.none, lc_off, FakeSM('carry', 1))
    assert not modeld_v2.is_blendful_active(log.Desire.none, lc_off, FakeSM('carry', -1))
    assert not modeld_v2.is_blendful_active(log.Desire.turnRight, lc_off, FakeSM('none', 0))
    assert not modeld_v2.is_blendful_active(log.Desire.none, log.LaneChangeState.laneChangeStarting, FakeSM('none', 0))

    src = Path(modeld_v2.__file__).read_text()
    assert "blendful_active = is_blendful_active(desire, DH.lane_change_state, sm)" in src
    assert "carry_from_sm(sm) == 0" in src
    assert "model.get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego, lane_keep," in src
