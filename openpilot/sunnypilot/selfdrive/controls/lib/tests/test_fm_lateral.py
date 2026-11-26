"""FM Pilot: FmLateral (fm_lateral.py) runs Lane Guard only when it is the Lane Keeping choice, on top of the request."""
from types import SimpleNamespace

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.lane_guard import TUNES
from openpilot.selfdrive.controls.tests.test_lane_guard import V_EGO, make_model
from openpilot.sunnypilot.selfdrive.controls.lib.fm_lane_keep_mode import MODE_BLENDFUL, MODE_CENTERING, MODE_GUARD, MODE_OFF
from openpilot.sunnypilot.selfdrive.controls.lib.fm_lateral import FmLateral

CP = SimpleNamespace(brand="rivian")
CLOSE_LEFT = make_model(-(1.3 - TUNES["rivian"].half_width + 0.05), lane_width=2.6)  # R1S body edge 5 cm over the left line


def cs(pressed=False):
  return SimpleNamespace(vEgo=V_EGO, steeringPressed=pressed, leftBlinker=False, rightBlinker=False)


def drive(mode, steps=500, lat_active=True, pressed=False, centering=False, desired=0.001):
  params = Params()
  params.put("FM_LaneKeepMode", mode, block=True)
  lat = FmLateral(CP, params)
  for _ in range(steps):
    out = lat.update(desired, CLOSE_LEFT, cs(pressed), lat_active, centering)
  return out - desired, lat


class TestFmLateral(OpenpilotTestCase):
  def test_guard_pushes_away_only_in_guard_mode(self):
    assert drive(MODE_GUARD)[0] > 0.0
    for mode in (MODE_OFF, MODE_CENTERING, MODE_BLENDFUL):
      assert drive(mode)[0] == 0.0, mode

  def test_other_modes_skip_the_update(self):
    _, lat = drive(MODE_OFF, steps=5)
    assert lat.lane_guard_resting

  def test_lane_centering_wins(self):
    assert drive(MODE_GUARD, centering=True)[0] == 0.0

  def test_not_active_passes_through(self):
    assert drive(MODE_GUARD, lat_active=False)[0] == 0.0

  def test_steering_pressed_fades_out(self):
    assert drive(MODE_GUARD, pressed=True)[0] == 0.0


class _FakeSM(dict):
  def __init__(self, turn_dir, queue_hold=False):
    super().__init__(modelDataV2SP=SimpleNamespace(laneTurnDirection=turn_dir),
                     fmModel=SimpleNamespace(queueGuard=queue_hold, turnSettlePhase='idle', turnSettle=0.0))
    self.seen = {'modelDataV2SP': True, 'fmModel': True}


class TestLaneKeepWaitsForTurnCarry(OpenpilotTestCase):
  def test_lane_centering_waits_for_carry_and_resumes_like_blinker(self):
    from openpilot.cereal import custom
    from openpilot.sunnypilot.selfdrive.controls.lib.lane_centering import LaneCenteringController
    from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_lane_centering import _model
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import PARAM_CARRY, PARAM_HOLD, SOURCE_CARRY

    TurnDirection = custom.ModelDataV2SP.TurnDirection
    md = _model(left=-1.5, right=2.1)  # clearly off center so Lane Centering pushes whenever active
    v_ego = 6.0  # above Lane Centering's 5 m/s min speed and inside Turn Assist carry speed (< 20 mph)

    def run(turn_dir, sign, carry_enabled, blinker_frames, carry_frames, after_frames):
      params = Params()
      params.put("FM_LaneKeepMode", MODE_CENTERING, block=True)
      params.put_bool(PARAM_HOLD, True, block=True)
      params.put_bool(PARAM_CARRY, carry_enabled, block=True)
      lc = LaneCenteringController(params)
      lc.get_params()
      lat = FmLateral(CP, params)

      lc_deltas, active_flags, sources = [], [], []
      total = blinker_frames + carry_frames + after_frames
      for i in range(total):
        blinker_on = i < blinker_frames
        in_turn = i < blinker_frames + carry_frames
        actual_k = sign * 0.05 if in_turn else 0.0
        direction = turn_dir if blinker_on else TurnDirection.none
        car_state = SimpleNamespace(vEgo=v_ego, steeringPressed=False, steeringTorque=0.0, aEgo=0.0,
                                    leftBlinker=(blinker_on and sign < 0), rightBlinker=(blinker_on and sign > 0))
        sm = _FakeSM(direction)
        # Exact controlsd.py call order: carry_active -> lane_centering.update -> fm_lateral.update
        carry = lat.carry_active(md, car_state, True, actual_k, sm)
        lc_out = lc.update(0.0, md, v_ego, bool(car_state.leftBlinker or car_state.rightBlinker or carry),
                           True, True, car_state.steeringPressed)
        lat.update(lc_out, md, car_state, True, lc.enabled, actual_k, sm)
        lc_deltas.append(lc_out)
        active_flags.append(carry)
        sources.append(lat.turn_assist.log_source)
      return lc_deltas, active_flags, sources

    for turn_dir, sign in ((TurnDirection.turnLeft, -1), (TurnDirection.turnRight, 1)):
      # 40 frames (0.4 s) blinker (~6.9 deg < 45 deg), 80 frames (0.8 s) carry, 150 frames (1.5 s) straight
      carry_deltas, carry_flags, carry_sources = run(turn_dir, sign, True, 40, 80, 150)
      # Reference: same total turn length (120 frames) with blinker staying on, then 150 frames straight
      blinker_deltas, _, _ = run(turn_dir, sign, False, 120, 0, 150)
      # Reference with carry disabled: blinker cancels at frame 40 and Lane Centering resumes immediately
      no_carry_deltas, no_carry_flags, no_carry_sources = run(turn_dir, sign, False, 40, 80, 150)

      # During the carry, Lane Centering adds nothing from frame 0 through the final carry frame
      assert all(carry_flags[40:120])
      assert all(s == SOURCE_CARRY for s in carry_sources[40:120])
      assert all(d == 0.0 for d in carry_deltas[40:120])
      # After the carry ends, Lane Centering resumes bit-for-bit like after a normal blinker turn
      assert not any(carry_flags[120:])
      assert carry_deltas[120:] == blinker_deltas[120:]
      assert carry_deltas[-1] > 0.0
      # With no carry, behavior is unchanged (Lane Centering activates right after blinker cancels)
      assert not any(no_carry_flags)
      assert all(s != SOURCE_CARRY for s in no_carry_sources)
      assert no_carry_deltas[119] > 0.0

  def test_lane_guard_waits_for_carry_and_keeps_cooldown(self):
    from openpilot.cereal import custom
    from openpilot.selfdrive.controls.lib.lane_guard import LANE_CHANGE_COOLDOWN
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import PARAM_CARRY, PARAM_HOLD, SOURCE_CARRY

    TurnDirection = custom.ModelDataV2SP.TurnDirection
    v_ego = 8.5  # > Lane Guard's 8.0 m/s min speed and < Turn Assist's 20 mph (8.94 m/s) cutoff
    close_edge = make_model(-(1.3 + 0.1 - TUNES["rivian"].half_width + 0.05), lane_width=2.6, curve=0.0, edge=0.1)
    centered = make_model(0.0, lane_width=2.6, curve=0.0, edge=1.0)

    def run_guard(turn_dir, sign, carry_enabled, blinker_frames, carry_frames, after_frames, model_during_carry, model_after):
      params = Params()
      params.put("FM_LaneKeepMode", MODE_GUARD, block=True)
      params.put_bool(PARAM_HOLD, True, block=True)
      params.put_bool(PARAM_CARRY, carry_enabled, block=True)
      lat = FmLateral(CP, params)

      guards, timers, sources = [], [], []
      total = blinker_frames + carry_frames + after_frames
      for i in range(total):
        blinker_on = i < blinker_frames
        in_turn = i < blinker_frames + carry_frames
        actual_k = sign * 0.05 if in_turn else 0.0
        direction = turn_dir if blinker_on else TurnDirection.none
        md = centered if blinker_on else (model_during_carry if in_turn else model_after)
        car_state = SimpleNamespace(vEgo=v_ego, steeringPressed=False, steeringTorque=0.0, aEgo=0.0,
                                    leftBlinker=(blinker_on and sign < 0), rightBlinker=(blinker_on and sign > 0))
        lat.update(0.0, md, car_state, True, False, actual_k, _FakeSM(direction))
        guards.append(lat.lane_guard.curvature)
        timers.append(lat.lane_guard.lane_change_timer)
        sources.append(lat.turn_assist.log_source)
      return guards, timers, sources

    for turn_dir, sign in ((TurnDirection.turnLeft, -1), (TurnDirection.turnRight, 1)):
      # 1. Lane lines: stays 0 during carry, respects normal 1.0 s cooldown after carry ends, resumes like after blinker
      carry_g, carry_t, carry_s = run_guard(turn_dir, sign, True, 20, 60, 220, CLOSE_LEFT, CLOSE_LEFT)
      blinker_g, blinker_t, _ = run_guard(turn_dir, sign, False, 80, 0, 220, CLOSE_LEFT, CLOSE_LEFT)
      no_carry_g, _, _ = run_guard(turn_dir, sign, False, 20, 60, 220, CLOSE_LEFT, CLOSE_LEFT)

      assert all(s == SOURCE_CARRY for s in carry_s[20:80])
      assert all(g == 0.0 for g in carry_g[20:80])
      assert all(t == LANE_CHANGE_COOLDOWN for t in carry_t[20:80])
      # Normal 1.0 s cooldown (100 frames) after carry ends, matching a normal blinker turn bit-for-bit
      assert all(g == 0.0 for g in carry_g[80:179])
      assert carry_g[80:] == blinker_g[80:]
      assert carry_t[80:] == blinker_t[80:]
      assert carry_g[-1] > 0.0
      # With no carry, cooldown starts at frame 20 (so it is already active well before frame 179)
      assert no_carry_g[150] > 0.0

      # 2. Road edges: also suppressed during carry (blinker=True alone would still guard road edges!)
      edge_carry_g, _, edge_carry_s = run_guard(turn_dir, sign, True, 20, 60, 60, close_edge, close_edge)
      edge_no_carry_g, _, _ = run_guard(turn_dir, sign, False, 20, 60, 60, close_edge, close_edge)
      assert all(s == SOURCE_CARRY for s in edge_carry_s[20:80])
      assert all(g == 0.0 for g in edge_carry_g[20:80])
      assert edge_carry_g[-1] > 0.0  # resumes after carry ends
      assert edge_no_carry_g[40] > 0.0  # unchanged when no carry

  def test_will_carry_matches_update_and_is_read_only(self):
    import numpy as np
    from openpilot.cereal import custom
    from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import (FmTurnAssist, PARAM_CARRY, PARAM_HOLD,
                                                                            SOURCE_CARRY)

    TurnDirection = custom.ModelDataV2SP.TurnDirection
    params = Params()
    params.put_bool(PARAM_HOLD, True, block=True)
    params.put_bool(PARAM_CARRY, True, block=True)
    ta = FmTurnAssist(params)
    rng = np.random.default_rng(42)
    dirs = (TurnDirection.none, TurnDirection.turnLeft, TurnDirection.turnRight)
    direction = TurnDirection.none
    saw_carry = False
    for _ in range(2000):
      if rng.random() < 0.05:
        direction = dirs[int(rng.integers(0, 3))]
      v = float(rng.uniform(0.0, 12.0))
      k = float(rng.normal(0.0, 0.05))
      pressed = bool(rng.random() < 0.1)
      torque = float(rng.normal(0.0, 0.8)) if pressed else 0.0
      lat_active = bool(rng.random() > 0.05)
      lc = bool(rng.random() < 0.02)
      qh = bool(rng.random() < 0.02)

      before = (ta.carry_sign, ta.carry_time, ta.carry_against, ta.carry_done)
      predicted = ta.will_carry(direction, v, k, pressed, torque, lat_active, lane_changing=lc, queue_hold=qh)
      assert (ta.carry_sign, ta.carry_time, ta.carry_against, ta.carry_done) == before
      ta.update(k, direction, v, k, pressed, lat_active, lane_changing=lc, steering_torque=torque, queue_hold=qh)
      actual = ta.log_source == SOURCE_CARRY
      assert predicted == actual
      saw_carry = saw_carry or actual
    assert saw_carry

  def test_controlsd_passes_carry_active_to_lane_centering(self):
    import ast
    from pathlib import Path
    import openpilot.selfdrive.controls as controls

    tree = ast.parse((Path(controls.__file__).parent / "controlsd.py").read_text())
    carry_calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and ast.unparse(n.func) == "self.fm_lateral.carry_active"]
    lc_calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and ast.unparse(n.func) == "self.lane_centering.update"]
    assert len(carry_calls) == 1 and len(lc_calls) == 1
    assert carry_calls[0].lineno < lc_calls[0].lineno
    assert "carry_active" in ast.unparse(lc_calls[0].args[3])
