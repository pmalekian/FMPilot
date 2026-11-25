"""FM Pilot: Settle After Turn > Early Right Push (FM_TurnSettleEarlyPush).

No recorded turn was replayed for this build (000000af wasn't at hand), so these use a small closed-loop turn simulator:
the truck turns with the desire on, straightens onto a straight road, then follows a model that only lines its heading up
with the road (no lines seen, so it holds whatever lateral position the turn left), plus Settle's curvature, through a
0.3 s steering lag. Lines, road edges and the plan are drawn in the truck's frame from where it is on that road.
"""
import math
import re
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import custom, log
from openpilot.selfdrive.controls.lib.lane_guard import MARGIN, TUNES
from openpilot.sunnypilot.modeld_v2 import fm_turn_settle as fts
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants
from openpilot.sunnypilot.modeld_v2.tests import fm_turn_settle_baseline as base

DT = fts.DT
X = np.array(ModelConstants.X_IDXS)
T = np.array(ModelConstants.T_IDXS)
HALF_LANE = 1.65  # m, a 3.3 m lane
MPH = 0.44704


def model_output(y, e, v, probs=(0.0, 0.0), stds=(0.5, 0.5), right_edge=None, edge_std=math.inf, swerve=0.0, plan_bend=0.0):
  """y: the truck's lateral position from the lane center (m, right positive); e: its heading minus the road's (rad,
  right positive). right_edge: the curb's lateral position from the lane center. swerve: the plan going left (m) from
  10 m on (around a parked car); plan_bend: the plan still curving toward a turn (1/m, right positive)."""
  lines = np.zeros((1, 4, 33, 2))
  for i, off in enumerate((-3 * HALF_LANE, -HALF_LANE, HALF_LANE, 3 * HALF_LANE)):
    lines[0, i, :, 0] = (off - y) - e * X
  lane_stds = np.ones((1, 4, 33, 2))
  lane_stds[0, 1, :, 0], lane_stds[0, 2, :, 0] = stds
  lane_probs = np.zeros((1, 8))
  lane_probs[0, 3], lane_probs[0, 5] = probs
  edges = np.zeros((1, 2, 33, 2))
  edges[0, 0, :, 0] = (-6.0 - y) - e * X
  edges[0, 1, :, 0] = ((right_edge if right_edge is not None else 6.0) - y) - e * X
  edge_stds = np.ones((1, 2, 33, 2))
  edge_stds[0, 0, :, 0], edge_stds[0, 1, :, 0] = math.inf, edge_std
  plan = np.zeros((1, 33, 15))
  px = max(v, 0.5) * T
  plan[0, :, 0] = px
  plan[0, :, 1] = -e * px - swerve * np.clip((px - 5.0) / 5.0, 0.0, 1.0) + plan_bend * px ** 2 / 2
  return {'lane_lines': lines, 'lane_lines_stds': lane_stds, 'lane_lines_prob': lane_probs, 'road_edges': edges,
          'road_edges_stds': edge_stds, 'plan': plan}


class Sim:
  """The truck after a turn: the model lines its heading up with the road, Settle adds its curvature, 0.3 s lag.
  align: how hard the model turns the truck back to the road's direction (1/m per rad): 0.2 aims along the road 10 m
  ahead (fights the push hard); 0.05 about 40 m ahead. How the real models respond is not known (no replay yet)."""

  def __init__(self, early=True, y0=0.0, v=4.0, align=0.2):
    self.s = fts.TurnSettle()
    self.early, self.y, self.e, self.v, self.align = early, y0, 0.0, v, align
    self.k = 0.0  # the truck's curvature (controlsState.curvature)
    self.turning = False  # y is held during the turn itself (it is only the lane position once on the new road)
    self.log = []

  def frame(self, desire=log.Desire.none, k_turn=None, pressed=False, trust=0.0, v=None, plan_bend=0.0, **out_kw):
    if v is not None:
      self.v = v
    out = model_output(self.y, self.e, self.v, plan_bend=plan_bend, **out_kw)
    model_k = k_turn if k_turn is not None else -self.align * self.e
    sm = {'controlsState': SimpleNamespace(curvature=self.k), 'carControl': SimpleNamespace(latActive=True),
          'carState': SimpleNamespace(steeringPressed=pressed)}
    settle = fts.settle_from_model(self.s, True, desire, log.LaneChangeState.off, self.v, sm, trust, out, self.early, model_k)
    wanted = model_k + settle
    self.k += (wanted - self.k) * DT / (0.3 + DT)
    self.e += self.k * self.v * DT
    if not self.turning:
      self.y += self.v * math.sin(self.e) * DT
    self.log.append(SimpleNamespace(settle=settle, y=self.y, e=self.e, push=self.s.push, limit=self.s.limit,
                                    phase=self.s.phase, guard=self.s.guard, active=self.s.active, desire=desire))
    return settle

  def turn(self, desire=log.Desire.turnRight, degrees=90.0, k=0.1, desire_after=0.0):
    """Turn `degrees` at curvature k with the desire on, unwinding over 1 s, then keep the desire on desire_after s."""
    sign = 1 if desire == log.Desire.turnRight else -1
    unwind = int(round(1.0 / DT))
    self.e = -sign * math.radians(degrees)  # heading against the new road; the turn (with the lag) brings it near 0
    self.turning = True
    # heading turned while unwinding (k falls linearly to 0): about half of k*v*1 s
    hold_rad = math.radians(degrees) - 0.5 * k * self.v * 1.0
    for _ in range(int(round(hold_rad / (k * self.v * DT)))):
      self.frame(desire, k_turn=sign * k, plan_bend=sign * k)
    for i in range(unwind):
      kk = k * (1.0 - (i + 1) / unwind)
      self.frame(desire, k_turn=sign * kk, plan_bend=sign * kk if abs(self.e) > 0.15 else 0.0)
    self.turning = False  # on the new road, whatever heading error the lag left (the model lines it up)
    for _ in range(int(round(desire_after / DT))):
      self.frame(desire)

  def run(self, seconds, **kw):
    return [self.frame(**kw) for _ in range(int(round(seconds / DT)))]


RIGHT, LEFT = log.Desire.turnRight, log.Desire.turnLeft


# ---- with the option off: exactly as before -------------------------------------------------------------------------

def _random_inputs(rng, n):
  """Random but turn-like input streams: desire stretches, curvature, speed, lines/edges flickering in and out."""
  desire = 0
  for _ in range(n):
    if rng.random() < 0.03:
      desire = int(rng.choice([0, 0, 1, -1]))
    k = desire * float(rng.uniform(0.0, 0.15)) if desire and rng.random() < 0.8 else float(rng.normal(0.0, 0.01))
    v = float(np.clip(rng.normal(6.0, 4.0), 0.0, 16.0))
    lines = rng.normal(0.0, 2.0, (4, 33)) + np.array([-5.0, -1.7, 1.7, 5.0])[:, None]
    edges = rng.normal(0.0, 2.0, (2, 33)) + np.array([-3.5, 3.5])[:, None]
    yield {'desire_sign': desire, 'lane_changing': bool(rng.random() < 0.02), 'v_ego': v, 'truck_curvature': k,
           'lat_active': bool(rng.random() < 0.97), 'steering_pressed': bool(rng.random() < 0.02),
           'blendful_trust': float(rng.choice([0.0, 0.0, rng.uniform(0.0, 1.0)])), 'x_idxs': X, 'lines_y': lines,
           'line_probs': rng.uniform(0.0, 0.7, 4), 'line_stds': rng.uniform(0.0, 1.5, 4), 'edges_y': edges,
           'edge_stds': rng.uniform(0.0, 1.5, 2), 'plan_x': X, 'plan_y': rng.normal(0.0, 1.0, 33).cumsum() * 0.1}


@pytest.mark.parametrize("seed", range(40))
def test_off_matches_the_code_before_it(seed):
  """Early Right Push off: TurnSettle gives bit-identical output and state to the pre-change module (a5f04fb)."""
  rng = np.random.default_rng(seed)
  old, new = base.TurnSettle(), fts.TurnSettle()
  enabled = True
  for i, kw in enumerate(_random_inputs(rng, 3000)):
    if rng.random() < 0.005:
      enabled = not enabled
    a = old.update(enabled, **kw)
    b = new.update(enabled, **kw, model_curvature=float(rng.normal(0.0, 0.02)))  # early defaults off
    assert a == b, (seed, i)
    assert (old.sign, old.turned, old.time_left, old.curvature, old.offset, old.active) == \
           (new.sign, new.turned, new.time_left, new.curvature, new.offset, new.active), (seed, i)


@pytest.mark.parametrize("seed", range(10))
def test_off_matches_through_settle_from_model(seed):
  """The same through modeld_v2's call, on random model outputs (incl. missing road edges)."""
  rng = np.random.default_rng(100 + seed)
  old, new = base.TurnSettle(), fts.TurnSettle()
  desire = log.Desire.none
  for _ in range(2000):
    if rng.random() < 0.03:
      desire = rng.choice([log.Desire.none, log.Desire.turnLeft, log.Desire.turnRight, log.Desire.laneChangeLeft])
    out = model_output(float(rng.normal(0, 0.8)), float(rng.normal(0, 0.05)), float(rng.uniform(0, 15)),
                       probs=tuple(rng.uniform(0, 0.7, 2)), stds=tuple(rng.uniform(0, 1.2, 2)),
                       right_edge=float(rng.uniform(1, 4)), edge_std=float(rng.uniform(0, 1.2)))
    if rng.random() < 0.1:
      del out['road_edges'], out['road_edges_stds']
    v = float(rng.uniform(0, 15))
    sm = {'controlsState': SimpleNamespace(curvature=float(rng.normal(0, 0.05))),
          'carControl': SimpleNamespace(latActive=bool(rng.random() < 0.97)),
          'carState': SimpleNamespace(steeringPressed=bool(rng.random() < 0.02))}
    trust = float(rng.choice([0.0, rng.uniform(0, 1)]))
    lcs = log.LaneChangeState.off if rng.random() < 0.98 else log.LaneChangeState.laneChangeStarting
    enabled = rng.random() < 0.99
    a = base.settle_from_model(old, enabled, desire, lcs, v, sm, trust, out)
    b = fts.settle_from_model(new, enabled, desire, lcs, v, sm, trust, out, False, float(rng.normal(0, 0.02)))
    assert a == b
    assert (old.curvature, old.offset) == (new.curvature, new.offset)


def test_off_logs_nothing_new():
  sim = Sim(early=False)
  sim.turn(LEFT)
  sim.run(6.0)
  assert all(f.push == 0.0 and f.limit == fts.LIMIT_NONE and f.phase == fts.PHASE_IDLE and f.guard == 0.0 for f in sim.log)


def test_setting_is_off_by_default():
  keys = (Path(__file__).parents[3] / "common" / "params_keys.h").read_text()
  assert re.search(r'\{"FM_TurnSettleEarlyPush", \{PERSISTENT \| BACKUP, BOOL, "0"\}\}', keys)


def test_settle_off_with_early_on_is_zero():
  s = fts.TurnSettle()
  out = model_output(0.0, 0.0, 4.0)
  sm = {'controlsState': SimpleNamespace(curvature=0.0), 'carControl': SimpleNamespace(latActive=True),
        'carState': SimpleNamespace(steeringPressed=False)}
  assert all(fts.settle_from_model(s, False, log.Desire.none, log.LaneChangeState.off, 4.0, sm, 0.0, out, True) == 0.0
             for _ in range(50))


def test_log_values_match_the_schema():
  limits = custom.FmModel.SettleLimit.schema.enumerants  # fm-dev-sync: logged in fmModel (was ModelDataV2SP.FmSettle*)
  phases = custom.FmModel.SettlePhase.schema.enumerants
  assert (limits['none'], limits['cap'], limits['center'], limits['curb'], limits['goingAround'], limits['hands'],
          limits['blendful'], limits['timeout'], limits['speed'], limits['other']) == \
         (fts.LIMIT_NONE, fts.LIMIT_CAP, fts.LIMIT_CENTER, fts.LIMIT_CURB, fts.LIMIT_YIELD, fts.LIMIT_HANDS,
          fts.LIMIT_BLENDFUL, fts.LIMIT_TIMEOUT, fts.LIMIT_SPEED, fts.LIMIT_OTHER)
  assert (phases['idle'], phases['push'], phases['centering'], phases['handedOff']) == \
         (fts.PHASE_IDLE, fts.PHASE_PUSH, fts.PHASE_CENTERING, fts.PHASE_HANDED_OFF)


def test_lane_guard_numbers():
  assert TUNES["rivian"].half_width == pytest.approx(1.007)
  assert MARGIN == pytest.approx(0.15)


# ---- the scenarios ----------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("desire", [LEFT, RIGHT])
def test_blind_push_stops_at_the_cap(desire):
  """No line seen after a left or a right turn: pushes right, never left, and stops at PUSH_MAX of its own motion.
  The truck coasts on a little after the push lets go, until the model has straightened it (here a model that takes
  ~40 m to): about 0.1 m, so at most ~0.5 m in all."""
  sim = Sim(y0=-0.6, v=6.7, align=0.05)  # left where the turn put it (like 000000af T5), 15 mph
  sim.turn(desire, k=0.06)
  ks = sim.run(fts.WINDOW)
  assert min(ks) >= 0.0
  capped = next(i for i, f in enumerate(sim.log) if f.limit == fts.LIMIT_CAP)
  after = [f.settle for f in sim.log[capped:]]
  assert all(b <= a for a, b in zip(after, after[1:], strict=False)) and after[-1] == 0.0  # stops pushing at the cap
  assert sim.log[capped].push == pytest.approx(fts.PUSH_MAX, abs=0.01)
  moved = max(f.y for f in sim.log) - (-0.6)  # what the truck really moved
  assert fts.PUSH_MAX - 0.05 <= moved <= fts.PUSH_MAX + 0.15
  # the push's own measure tracks the truck's real movement (reading high stops it early, the safe side)
  assert -0.06 < max(f.push for f in sim.log) - moved < 0.08
  assert all(f.phase in (fts.PHASE_PUSH, fts.PHASE_IDLE) for f in sim.log)


@pytest.mark.parametrize("desire", [LEFT, RIGHT])
@pytest.mark.parametrize("align", [0.05, 0.1, 0.2])
@pytest.mark.parametrize("v", [4.0, 10.0])
def test_blind_push_is_bounded_whatever_the_model_does(desire, align, v):
  """However hard the model turns the truck back to the road (aiming 40 down to 10 m ahead) and at 9 or 22 mph: only
  ever right, and the truck never ends up more than ~0.5 m right of where the turn left it."""
  sim = Sim(y0=-0.6, v=v, align=align)
  sim.turn(desire, k=0.1 if v < 5 else 0.05)
  ks = sim.run(fts.WINDOW)
  assert min(ks) >= 0.0 and max(ks) > 0.0
  moved = max(f.y for f in sim.log) - (-0.6)
  assert 0.15 <= moved <= fts.PUSH_MAX + 0.15


@pytest.mark.parametrize("desire", [LEFT, RIGHT])
@pytest.mark.parametrize("y0", [0.0, 0.2, 0.5])
def test_centered_or_right_turn_ending_gets_no_push(desire, y0):
  """Like 000000af Turns 2 and 4: a 30%+ line shows the truck centered or right of it as the turn ends: no push right."""
  sim = Sim(y0=y0)
  sim.turn(desire)
  ks = sim.run(fts.WINDOW, probs=(0.35, 0.35))
  assert max(ks) <= 0.0  # never toward the curb (after a right turn the centering may still ease it left)
  assert sim.log[-1].y <= y0 + 0.01
  assert any(f.limit == fts.LIMIT_CENTER for f in sim.log)
  if desire == LEFT:
    assert all(k == 0.0 for k in ks)  # after a left it never pushes left


def test_line_seen_late_shows_centered_ends_the_push():
  """The line shows up 1 s into the push and says centered: the push stops for this turn and doesn't come back."""
  sim = Sim(y0=0.0)
  sim.turn(RIGHT)
  sim.run(1.0)
  pushed = sim.log[-1].y
  ks = sim.run(0.5, probs=(0.35, 0.35))
  ks += sim.run(3.0)  # the line is lost again
  assert sim.s.push_done and sim.s.limit == fts.LIMIT_CENTER
  assert ks[-1] == 0.0
  assert max(f.push for f in sim.log) < 0.25
  assert pushed < 0.15


def test_sure_curb_stops_the_push_and_steers_back():
  """A sure curb 1.6 m right of the truck: the push stops 1.4 m from it. One 1.25 m right: steers back left."""
  sim = Sim(y0=0.0)
  sim.turn(LEFT)
  sim.run(fts.WINDOW, right_edge=1.6, edge_std=0.3)
  assert max(f.y for f in sim.log) <= 1.6 - fts.EDGE_CLEARANCE + 0.03
  assert any(f.limit == fts.LIMIT_CURB for f in sim.log)

  sim = Sim(y0=0.0)
  sim.turn(RIGHT)
  ks = sim.run(2.0, right_edge=1.05, edge_std=0.3)  # the body's right side at 1.007 - 0.04 m: 0.08 m from the curb
  assert min(ks) < 0.0  # steering back left
  assert min(f.guard for f in sim.log) < 0.0
  assert sim.log[-1].y < 0.0
  assert sim.s.limit == fts.LIMIT_CURB

  # an unsure curb (std 0.8 m) is ignored, as in Settle
  sim = Sim(y0=0.0)
  sim.turn(LEFT)
  sim.run(fts.WINDOW, right_edge=1.6, edge_std=0.8)
  assert max(f.y for f in sim.log) > 0.2 + 0.05
  assert not any(f.limit == fts.LIMIT_CURB for f in sim.log)


def test_curb_guard_works_in_any_lane_keep_mode():
  """The guard lives in Settle: no FM_LaneKeepMode or Lane Guard gate (no 70% lines, works under 8 m/s)."""
  sim = Sim(y0=0.0, v=2.5)
  sim.turn(RIGHT)
  ks = sim.run(2.0, right_edge=1.05, edge_std=0.4)
  assert min(ks) < 0.0


def test_parked_car_plan_swing_yields():
  """The model's plan swings 0.6 m left around a parked car: the push holds; once past it, the push carries on."""
  sim = Sim(y0=-0.3)
  sim.turn(RIGHT)
  sim.run(0.5)
  before = sim.log[-1].settle
  assert before > 0.0
  held = sim.run(1.5, swerve=0.6)
  assert held[-1] == 0.0
  assert sim.s.limit == fts.LIMIT_YIELD
  assert all(b <= a + 1e-12 for a, b in zip(held, held[1:], strict=False))  # only ever lets go meanwhile
  after = sim.run(2.0)
  assert max(after) > 0.0
  # a small swing (0.1 m) is not going around anything
  sim = Sim(y0=-0.3)
  sim.turn(RIGHT)
  sim.run(0.5)
  assert sim.run(1.0, swerve=0.1)[-1] > 0.0


def test_hand_on_the_wheel_cancels_at_once():
  sim = Sim(y0=-0.6)
  sim.turn(LEFT)
  sim.run(1.0)
  assert sim.log[-1].settle > 0.0
  assert sim.frame(pressed=True) == 0.0
  assert sim.s.limit == fts.LIMIT_HANDS
  assert all(k == 0.0 for k in sim.run(4.0))


@pytest.mark.parametrize("mph", [3.0, 3.5, 5.0, 8.0, 11.0, 15.0, 25.0])
def test_push_stays_under_the_lateral_accel_cap(mph):
  """Crawl-speed exits too: under ~11 mph never more than 0.1 m/s^2 (nor 0.01 1/m); above it the old 0.004 1/m."""
  v = mph * MPH + 1e-3
  sim = Sim(y0=-1.0, v=v)
  sim.turn(LEFT, k=0.15 if mph < 10 else 0.05)
  ks = np.array(sim.run(fts.WINDOW))
  assert ks.max() > 0.0
  assert np.all(np.abs(ks) <= fts.low_speed_cap(v) + 1e-12)
  if v ** 2 * fts.MAX_CURVATURE <= fts.LOW_SPEED_LAT_ACCEL:
    assert np.all(np.abs(ks) * v ** 2 <= fts.LOW_SPEED_LAT_ACCEL + 1e-9)


def test_low_speed_cap_values():
  assert fts.low_speed_cap(15 * MPH) == fts.MAX_CURVATURE  # old cap above ~11 mph
  assert fts.low_speed_cap(8 * MPH) == pytest.approx(0.1 / (8 * MPH) ** 2)  # 0.0078, about twice the old cap
  assert fts.low_speed_cap(3 * MPH) == fts.MAX_CURVATURE_LOW


def test_below_3_mph_waits():
  sim = Sim(y0=-0.6, v=2.5 * MPH)
  sim.turn(LEFT, k=0.15)
  assert all(k == 0.0 for k in sim.run(2.0))
  assert max(sim.run(2.0, v=4.0 * MPH)) > 0.0


def test_push_then_centering_then_blendful():
  """No line: push. A line shows the truck 0.7 m left of center: centering. Blendful's trust reaches 0.5: handed off."""
  sim = Sim(y0=-0.9)
  sim.turn(LEFT)
  sim.run(1.0)
  assert sim.s.phase == fts.PHASE_PUSH and sim.log[-1].settle > 0.0
  centering = sim.run(1.0, probs=(0.35, 0.35))
  assert sim.s.phase == fts.PHASE_CENTERING and sim.s.offset > fts.DEADBAND and centering[-1] > 0.0
  k = sim.log[-1].settle
  fade = sim.run(1.0, probs=(0.9, 0.9), trust=fts.HANDOVER_TRUST)
  assert sim.s.limit == fts.LIMIT_BLENDFUL and not sim.s.active
  assert fade[0] < k and fade[-1] == 0.0
  assert fts.PHASE_HANDED_OFF in [f.phase for f in sim.log]
  assert sim.s.phase == fts.PHASE_IDLE
  assert [p for i, p in enumerate(f.phase for f in sim.log) if i == 0 or p != sim.log[i - 1].phase] == \
         [fts.PHASE_IDLE, fts.PHASE_PUSH, fts.PHASE_CENTERING, fts.PHASE_HANDED_OFF, fts.PHASE_IDLE]


def test_starts_when_the_truck_straightens_not_at_the_desire_end():
  """The blinker stays on 2 s after the turn: the early push starts while it is still on; the classic waits for it."""
  early, classic = Sim(y0=-0.6, v=4.0), Sim(early=False, y0=-0.6, v=4.0)
  for sim in (early, classic):
    sim.turn(LEFT, desire_after=2.0)
    sim.run(2.0)
  first = next(i for i, f in enumerate(early.log) if f.settle > 0.0)
  assert early.log[first].desire == LEFT
  assert not any(f.settle != 0.0 for f in classic.log if f.desire == LEFT)


def test_no_start_while_still_turning():
  """45 deg turned but the wheel still at the turn's curvature, or the plan still bending into it: not yet."""
  sim = Sim(y0=-0.6)
  sim.turn(RIGHT, degrees=50.0, desire_after=0.0)
  for _ in range(40):  # straight wheel but the plan still bends 0.1 1/m into the turn
    sim.frame(RIGHT, k_turn=0.0, plan_bend=0.1)
  assert all(f.settle == 0.0 for f in sim.log)
  assert not sim.s.settled


def test_small_turn_does_nothing():
  sim = Sim(y0=-0.6)
  sim.turn(LEFT, degrees=30.0, desire_after=1.0)
  assert all(k == 0.0 for k in sim.run(4.0))


def test_new_blinker_ends_the_window():
  sim = Sim(y0=-0.6)
  sim.turn(LEFT)
  sim.run(1.0)
  assert sim.log[-1].settle > 0.0
  sim.run(0.5, desire=RIGHT)
  assert not sim.s.settled and sim.s.time_left == 0.0
  assert abs(sim.log[-1].settle) < sim.log[-11].settle


def test_switching_the_option_mid_window_fades():
  sim = Sim(y0=-0.6)
  sim.turn(LEFT)
  sim.run(1.0)
  k = sim.log[-1].settle
  sim.early = False
  ks = sim.run(1.0)
  assert 0.0 <= ks[0] < k
  assert k - ks[0] <= 2.0 / sim.v ** 2 * DT + 1e-12  # Blendful's release rate, no jump
  assert ks[-1] == 0.0
