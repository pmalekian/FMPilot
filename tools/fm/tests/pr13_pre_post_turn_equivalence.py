#!/usr/bin/env python3
"""FM Pilot plan PR 13: same-input equivalence of Pre/Post Turn Behavior (Queue Guard, Turn Pulse on Pull-Away, Settle
After Turn and its Early Right Push) against fm-dev-nav-settle ca08aac9.

REF is a folder with ca08aac9's fm_queue_guard.py, fm_turn_pulse.py, fm_turn_settle.py and fm_turn_assist.py (extracted
with `git show ca08aac9:<path>`). Every comparison is exact (==) on the output and every logged field and state, step by
step, with the same random and constructed inputs and every toggle combination:

 1. Queue Guard module: hold out, holding / sign / turned.
 2. Turn Pulse module: the model's desire, the stop memory.
 3. DesireHelper (fm-dev-sync, both on it) against ca08aac9's DesireHelper order (lane-turn desire, Queue Guard hides it,
    Turn Pulse on the result; its dropped Road Edge Turn Desire and Map Turn Desire off): laneTurnDirection, the hold
    (fmModel.queueGuard), the desire and the model's desire. Constructed: queued at a light with the signal on, creeps,
    the car ahead pulling away / leaving, 10 mph, steering the turn, pull-away straight and after a turn.
 4. Settle After Turn (+ Early Right Push), no carry: TurnSettle.update on random streams and settle_from_model on random
    model outputs; output, curvature, offset, push, limit, phase, guard and the internal state. Constructed closed-loop
    turns: settle after left / right with and without lines, the blind push, a curb, a parked car, hands, Blendful taking
    over, a lane change, 30 mph, a small turn, a new blinker.
 5. Settle with a carry (fm-dev-sync change, by design): the carried turn against ca08aac9 fed the desire kept on through
    the carry (expected identical), a carry 1-3 frames late (differences counted: only the gap frames' degrees), and
    against ca08aac9 fed the real signals (differences counted: a window after the carried turn, none in ca08aac9).
 6. Turn Assist (Turn Hold / Turn Lead / carry, every toggle combination) fed by the DesireHelper through queues: the new
    module also gets the hold; the only differences allowed are steps where ca08aac9 carried a turn out of a hold.
 7. Mutation check: each toggle changed on the new side only must show up.

  python tools/fm/tests/pr13_pre_post_turn_equivalence.py REF [n_random] [combo_stride]
"""
import importlib.util
import math
import random
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.cereal import custom, log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.sunnypilot.modeld_v2 import fm_turn_settle as new_settle
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib import fm_queue_guard as new_qg, fm_turn_pulse as new_tp
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController, LANE_CHANGE_SPEED_MIN

TD = custom.ModelDataV2SP.TurnDirection
TURN_DESIRES = {TD.none: log.Desire.none, TD.turnLeft: log.Desire.turnLeft, TD.turnRight: log.Desire.turnRight}
X = np.array(ModelConstants.X_IDXS)
T = np.array(ModelConstants.T_IDXS)
SETTLE_FIELDS = ("curvature", "offset", "push", "limit", "phase", "guard")
SETTLE_STATE = ("sign", "turned", "time_left", "active", "early", "peak", "straight_time", "prev_desire_sign", "settled",
                "psi", "road", "y", "push_done", "yielding")
COUNT: dict[str, int] = {}
PHASES = {v: k for k, v in custom.FmModel.SettlePhase.schema.enumerants.items()}
LIMITS = {v: k for k, v in custom.FmModel.SettleLimit.schema.enumerants.items()}


def _load(path, name):
  spec = importlib.util.spec_from_file_location(name, path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def bump(key, n=1):
  COUNT[key] = COUNT.get(key, 0) + n


class FakeParams:
  def __init__(self, values):
    self.values = dict(values)

  def get_bool(self, key):
    return bool(self.values.get(key))

  def get(self, key, return_default=False):
    return self.values.get(key)


def check(a, b, what):
  if a != b:
    raise AssertionError(f"{what}: {a!r} vs {b!r}")


# ---- 1. Queue Guard module ------------------------------------------------------------------------------------------

def queue_guard_random(ref, rng, n):
  steps = 0
  for _ in range(n):
    on = rng.random() < 0.8
    po, pn = FakeParams({ref.PARAM: on}), FakeParams({new_qg.PARAM: on})
    a, b = ref.FmQueueGuard(po), new_qg.FmQueueGuard(pn)
    sign, v, d = rng.choice([1, -1]), 0.0, 5.0
    for _ in range(rng.randint(100, 800)):
      if rng.random() < 0.02:
        sign = rng.choice([1, -1, 0, 0])
      if rng.random() < 0.003:
        po.values[ref.PARAM] = pn.values[new_qg.PARAM] = not po.values[ref.PARAM]  # switched while driving
      v = min(max(v + rng.gauss(0, 0.3), 0.0), 7.0) if rng.random() > 0.05 else rng.choice([0.0, 0.0, 0.2, 4.5, 6.0])
      d = min(max(d + rng.gauss(0, 0.5), 0.5), 20.0) if rng.random() > 0.05 else rng.uniform(0.5, 20.0)
      present = rng.random() > 0.1
      dist = d if present else rng.choice([math.inf, 0.0, float('nan')])
      k = sign * rng.choice([0.0, 0.0, rng.uniform(0, 0.15)]) + rng.gauss(0, 0.01)
      x = a.update(sign, v, k, DT_MDL, present, dist)
      y = b.update(sign, v, k, DT_MDL, present, dist)
      check((x, a.holding, a.sign, a.turned, a.enabled), (y, b.holding, b.sign, b.turned, b.enabled), "queue guard")
      bump("qg hold steps", int(y))
      steps += 1
  return steps


# ---- 2. Turn Pulse module -------------------------------------------------------------------------------------------

def turn_pulse_random(ref, rng, n):
  steps = 0
  desires = [log.Desire.none, log.Desire.turnLeft, log.Desire.turnRight, log.Desire.laneChangeLeft,
             log.Desire.laneChangeRight, log.Desire.keepLeft]
  for _ in range(n):
    on = rng.random() < 0.8
    po, pn = FakeParams({ref.PARAM: on}), FakeParams({new_tp.PARAM: on})
    a, b = ref.FmTurnPulse(po), new_tp.FmTurnPulse(pn)
    d, v = rng.choice(desires), 0.0
    for _ in range(rng.randint(100, 800)):
      if rng.random() < 0.02:
        d = rng.choice(desires)
      if rng.random() < 0.003:
        po.values[ref.PARAM] = pn.values[new_tp.PARAM] = not po.values[ref.PARAM]
      v = min(max(v + rng.gauss(0, 0.15), 0.0), 3.0) if rng.random() > 0.05 else rng.choice([0.0, 0.05, 0.1, 0.3, 1.0])
      x, y = a.update(d, v), b.update(d, v)
      check((x, a.stopped, a.enabled), (y, b.stopped, b.enabled), "turn pulse")
      bump("tp pulses", int(y == log.Desire.none and d != log.Desire.none))
      steps += 1
  return steps


# ---- 3. DesireHelper ------------------------------------------------------------------------------------------------

def cs(v, left=False, right=False):
  return SimpleNamespace(vEgo=v, leftBlinker=left, rightBlinker=right, steeringPressed=False, steeringTorque=0.0,
                         leftBlindspot=False, rightBlindspot=False, brakePressed=False)


def lead(d):
  return None if d is None else SimpleNamespace(present=d is not False, dRel=float(d) if d is not False else 0.0)


class RefDH:
  """ca08aac9's DesireHelper order for turns, with its Road Edge / Map Turn Desires (dropped) off."""

  def __init__(self, ref_qg, ref_tp, qg_on, tp_on):
    self.ltc = LaneTurnController(SimpleNamespace())
    self.ltc.enabled, self.ltc.lane_turn_value = True, LANE_CHANGE_SPEED_MIN
    self.qg = ref_qg.FmQueueGuard(FakeParams({ref_qg.PARAM: qg_on}))
    self.tp = ref_tp.FmTurnPulse(FakeParams({ref_tp.PARAM: tp_on}))

  def update(self, carstate, lead_msg, curvature):
    self.ltc.update_lane_turn(False, False, carstate.leftBlinker, carstate.rightBlinker, carstate.vEgo)
    direction = self.ltc.get_turn_direction()
    sign = 1 if direction == TD.turnRight else (-1 if direction == TD.turnLeft else 0)
    present = bool(lead_msg is not None and lead_msg.present)
    self.hold = self.qg.update(sign, carstate.vEgo, float(curvature), DT_MDL, present,
                               float(lead_msg.dRel) if present else float('inf'))
    self.direction = TD.none if self.hold else direction
    self.desire = TURN_DESIRES[self.direction]
    self.model_desire = self.tp.update(self.desire, carstate.vEgo)


def new_dh(qg_on, tp_on):
  dh = DesireHelper()
  dh.lane_turn_controller.enabled, dh.lane_turn_controller.lane_turn_value = True, LANE_CHANGE_SPEED_MIN
  dh.lane_turn_controller.read_params = lambda: None
  dh.fm_queue_guard.params = FakeParams({new_qg.PARAM: qg_on})
  dh.fm_turn_pulse.params = FakeParams({new_tp.PARAM: tp_on})
  dh.fm_queue_guard.read_params()
  dh.fm_turn_pulse.read_params()
  return dh


class DHPair:
  def __init__(self, ref_qg, ref_tp, qg_on, tp_on):
    self.ref, self.new = RefDH(ref_qg, ref_tp, qg_on, tp_on), new_dh(qg_on, tp_on)
    self.steps = 0
    self.trace = []  # (laneTurnDirection, hold) per frame, for section 6

  def step(self, v, left=False, right=False, d=None, k=0.0):
    c, ld = cs(v, left, right), lead(d)
    self.ref.update(c, ld, k)
    self.new.update(c, v > 0.3, 0.0, lead=ld, curvature=k)
    n = self.new
    check((self.ref.direction, self.ref.hold, self.ref.desire, self.ref.model_desire),
          (n.lane_turn_direction, n.fm_queue_hold, n.desire, n.model_desire), f"desire helper step {self.steps}")
    check(n.lane_change_state, log.LaneChangeState.off, "no lane change below 20 mph")
    self.steps += 1
    self.trace.append((v, k, n.lane_turn_direction, n.fm_queue_hold))
    bump("dh hold frames", int(n.fm_queue_hold))
    bump("dh model pulses", int(n.model_desire != n.desire))

  def run(self, n, **kw):
    for _ in range(n):
      self.step(**kw)


def dh_queue_cases(p, side):
  """Queued at a light with the signal on, every way the wait ends; pull-away straight and after a turn."""
  b = {"left": side < 0, "right": side > 0}
  s = side
  for end in ({"d": 12.0, "v": 1.5}, {"d": False, "v": 0.0}, {"d": None, "v": 0.0}, {"d": 6.0, "v": 4.6},
              {"d": 6.0, "v": 2.0, "k": s * 0.1}):
    p.run(20, v=3.0, d=15.0, **b)                       # approaching, the car ahead stopping
    p.run(10, v=1.0, d=9.0, k=s * 0.01, **b)            # creeping up behind it
    p.run(60, v=0.0, d=3.0, **b)                        # stopped at the light, signal on
    p.run(20, v=1.5, d=5.0, k=s * 0.05, **b)            # the queue creeps
    p.run(30, v=0.0, d=3.5, **b)
    p.run(5, v=1.0, d=8.9, **b)
    e = dict(end)
    p.run(30, v=e.pop("v"), d=e.pop("d"), k=e.pop("k", s * 0.08), **b)  # released
    p.run(60, v=3.0, d=20.0, k=s * 0.12, **b)           # the turn itself
    p.run(40, v=5.0, d=None)                            # blinker off, straight
  # stopped mid-turn behind a car: never held
  p.run(40, v=3.0, d=None, k=s * 0.12, **b)
  p.run(40, v=0.0, d=3.0, **b)
  p.run(20, v=1.0, d=12.0, k=s * 0.1, **b)
  p.run(20, v=4.0, d=None)
  # pull-away straight (no signal) and after a turn with a long wait (Turn Pulse)
  p.run(40, v=0.0, d=None)
  p.run(20, v=1.0, d=None)
  p.run(20, v=2.0, d=None, **b)
  p.run(200, v=0.0, d=None, **b)
  p.run(5, v=0.2, d=None, **b)
  p.run(30, v=1.5, d=None, k=s * 0.1, **b)
  p.run(20, v=4.0, d=None)
  # the blinker cancelling at turn-in while the truck still turns (a carry for Turn Assist, no hold)
  p.run(20, v=3.0, d=None, k=s * 0.1, **b)
  p.run(30, v=3.0, d=None, k=s * 0.1)
  p.run(20, v=3.0, d=None)
  # blinker flicker while stopped behind a car
  for i in range(40):
    p.step(v=0.0, d=3.0, left=b["left"] and i % 7 != 0, right=b["right"] and i % 7 != 0)
  p.run(20, v=1.0, d=12.0, **b)


def dh_random(ref_qg, ref_tp, rng, n):
  steps = 0
  for _ in range(n):
    p = DHPair(ref_qg, ref_tp, rng.random() < 0.7, rng.random() < 0.7)
    blink, v, d = rng.choice([None, "l", "r"]), 0.0, 5.0
    for _ in range(rng.randint(100, 600)):
      if rng.random() < 0.02:
        blink = rng.choice([None, "l", "r", "l", "r"])
      v = min(max(v + rng.gauss(0, 0.3), 0.0), 8.8) if rng.random() > 0.05 else rng.choice([0.0, 0.0, 0.2, 4.5, 6.0])
      d = min(max(d + rng.gauss(0, 0.5), 0.5), 25.0)
      dd = d if rng.random() > 0.1 else rng.choice([None, False])
      s = 1 if blink == "r" else -1 if blink == "l" else 0
      k = s * rng.choice([0.0, rng.uniform(0, 0.15)]) + rng.gauss(0, 0.005)
      p.step(v, left=blink == "l", right=blink == "r", d=dd, k=k)
    steps += p.steps
  return steps


# ---- 4./5. Settle After Turn ----------------------------------------------------------------------------------------

def settle_fields(s):
  return tuple(getattr(s, f) for f in SETTLE_FIELDS)


def settle_state(s):
  return tuple(getattr(s, f, None) for f in SETTLE_STATE)


def random_streams(rng, n):
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


def settle_random(ref, rng, n, frames):
  steps = 0
  for i in range(n):
    a, b = ref.TurnSettle(), new_settle.TurnSettle()
    enabled, early = True, i % 2 == 1
    for kw in random_streams(rng, frames):
      if rng.random() < 0.005:
        enabled = not enabled
      if rng.random() < 0.003:
        early = not early
      mk = float(rng.normal(0.0, 0.02))
      x = a.update(enabled, **kw, early=early, model_curvature=mk)
      y = b.update(enabled, **kw, early=early, model_curvature=mk)  # no carry
      check((x, settle_fields(a), settle_state(a)), (y, settle_fields(b), settle_state(b)), f"settle random {i}")
      bump("settle active frames", int(b.active))
      bump(f"settle phase {PHASES[b.phase]}", int(b.phase != 0))
      bump(f"settle limit {LIMITS[b.limit]}", int(b.limit != 0))
      steps += 1
  return steps


def model_output(y, e, v, probs=(0.0, 0.0), stds=(0.5, 0.5), right_edge=None, edge_std=math.inf, swerve=0.0, plan_bend=0.0):
  """As test_fm_turn_settle_early.model_output: the truck y m right of the lane center, heading e rad right of the road."""
  half = 1.65
  lines = np.zeros((1, 4, 33, 2))
  for i, off in enumerate((-3 * half, -half, half, 3 * half)):
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


class FmControlsSM(dict):
  """modeld_v2's SubMaster as settle_from_model reads it, with controlsd's fmControls carry when given."""

  def __init__(self, k, pressed, lat=True, carry=0):
    super().__init__(controlsState=SimpleNamespace(curvature=k), carControl=SimpleNamespace(latActive=lat),
                     carState=SimpleNamespace(steeringPressed=pressed))
    ta = custom.FmControls.new_message().turnAssist
    if carry:
      ta.sign, ta.source = carry, 'carry'
    self['fmControls'] = SimpleNamespace(turnAssist=ta)
    self.seen = {'fmControls': True}


def settle_from_model_random(ref, rng, n, frames):
  steps = 0
  desires = [log.Desire.none, log.Desire.turnLeft, log.Desire.turnRight, log.Desire.laneChangeLeft]
  for i in range(n):
    a, b = ref.TurnSettle(), new_settle.TurnSettle()
    desire, early = log.Desire.none, bool(i % 2)
    for _ in range(frames):
      if rng.random() < 0.03:
        desire = rng.choice(desires)
      v = float(rng.uniform(0, 15))
      out = model_output(float(rng.normal(0, 0.8)), float(rng.normal(0, 0.05)), v, probs=tuple(rng.uniform(0, 0.7, 2)),
                         stds=tuple(rng.uniform(0, 1.2, 2)), right_edge=float(rng.uniform(1, 4)),
                         edge_std=float(rng.uniform(0, 1.2)), swerve=float(rng.choice([0.0, rng.uniform(0, 1.0)])),
                         plan_bend=float(rng.normal(0, 0.02)))
      if rng.random() < 0.1:
        del out['road_edges'], out['road_edges_stds']
      sm = FmControlsSM(float(rng.normal(0, 0.05)), bool(rng.random() < 0.02), bool(rng.random() < 0.97))
      trust = float(rng.choice([0.0, rng.uniform(0, 1)]))
      lcs = log.LaneChangeState.off if rng.random() < 0.98 else log.LaneChangeState.laneChangeStarting
      enabled = rng.random() < 0.99
      mk = float(rng.normal(0, 0.02))
      x = ref.settle_from_model(a, enabled, desire, lcs, v, sm, trust, out, early, mk)
      y = new_settle.settle_from_model(b, enabled, desire, lcs, v, sm, trust, out, early, mk)
      check((x, settle_fields(a)), (y, settle_fields(b)), f"settle_from_model random {i}")
      steps += 1
  return steps


class SettlePair:
  """Closed-loop truck after a turn (as test_fm_turn_settle_early.Sim), the new module driving the loop; ca08aac9 fed
  the same inputs. ref_desire: what ca08aac9 is fed as the desire (default: the same as the new side)."""

  def __init__(self, ref, early, strict=True, v=4.0, align=0.2, y0=0.0):
    self.a, self.b, self.ref = ref.TurnSettle(), new_settle.TurnSettle(), ref
    self.early, self.strict, self.v, self.align = early, strict, v, align
    self.y, self.e, self.k, self.turning = y0, 0.0, 0.0, False
    self.steps = self.diffs = self.out_diffs = 0
    self.max_turned_gap = 0.0

  def frame(self, desire=log.Desire.none, k_turn=None, pressed=False, trust=0.0, v=None, plan_bend=0.0, carry=0,
            ref_desire=None, lane=False, lat=True, **out_kw):
    if v is not None:
      self.v = v
    out = model_output(self.y, self.e, self.v, plan_bend=plan_bend, **out_kw)
    mk = k_turn if k_turn is not None else -self.align * self.e
    lcs = log.LaneChangeState.laneChangeStarting if lane else log.LaneChangeState.off
    x = self.ref.settle_from_model(self.a, True, desire if ref_desire is None else ref_desire, lcs, self.v,
                                   FmControlsSM(self.k, pressed, lat), trust, out, self.early, mk)
    y = new_settle.settle_from_model(self.b, True, desire, lcs, self.v, FmControlsSM(self.k, pressed, lat, carry), trust,
                                     out, self.early, mk)
    self.steps += 1
    if (x, settle_fields(self.a), settle_state(self.a)) != (y, settle_fields(self.b), settle_state(self.b)):
      self.diffs += 1
      self.out_diffs += (x, settle_fields(self.a)) != (y, settle_fields(self.b))
      if self.a.sign == self.b.sign != 0:
        self.max_turned_gap = max(self.max_turned_gap, abs(self.a.turned - self.b.turned))
      if self.strict:
        raise AssertionError(f"settle closed loop step {self.steps}: {(x, settle_fields(self.a), settle_state(self.a))} "
                             f"vs {(y, settle_fields(self.b), settle_state(self.b))}")
    bump("closed-loop settle frames", int(y != 0.0))
    bump(f"settle phase {PHASES[self.b.phase]}", int(self.b.phase != 0))
    bump(f"settle limit {LIMITS[self.b.limit]}", int(self.b.limit != 0))
    self.k += (mk + y - self.k) * DT_MDL / (0.3 + DT_MDL)
    self.e += self.k * self.v * DT_MDL
    if not self.turning:
      self.y += self.v * math.sin(self.e) * DT_MDL
    return y

  def turn(self, desire, degrees=90.0, k=0.1, after=0.0, carry_from=None, gap=0, ref_keeps=False):
    """Turn `degrees` at curvature k, unwinding over 1 s. carry_from: the degrees after which the blinker cancels and
    Turn Assist carries the rest (gap: frames before the carry shows); ref_keeps: ca08aac9 gets the desire kept on
    instead of the carry."""
    sign = 1 if desire == log.Desire.turnRight else -1
    self.e, self.turning = -sign * math.radians(degrees), True
    hold = int(round((math.radians(degrees) - 0.5 * k * self.v) / (k * self.v * DT_MDL)))
    unwind = int(round(1.0 / DT_MDL))
    cancel = None if carry_from is None else int(round(math.radians(carry_from) / (k * self.v * DT_MDL)))
    for i in range(hold + unwind):
      kk = k if i < hold else k * (1.0 - (i - hold + 1) / unwind)
      carried = cancel is not None and i >= cancel
      d = log.Desire.none if carried else desire
      c = sign if carried and i >= cancel + gap else 0
      self.frame(d, k_turn=sign * kk, plan_bend=sign * kk if abs(self.e) > 0.15 else 0.0, carry=c,
                 ref_desire=desire if (carried and ref_keeps) else None)
    self.turning = False
    for _ in range(int(round(after / DT_MDL))):
      self.frame(desire)

  def run(self, seconds, **kw):
    for _ in range(int(round(seconds / DT_MDL))):
      self.frame(**kw)


MPH = CV.MPH_TO_MS


def settle_cases(p, desire):
  """Settle after a turn, every way it goes and ends."""
  other = log.Desire.turnLeft if desire == log.Desire.turnRight else log.Desire.turnRight
  for kw in ({}, {"probs": (0.35, 0.35)}, {"probs": (0.0, 0.4)}, {"probs": (0.4, 0.0)}, {"right_edge": 2.6, "edge_std": 0.3},
             {"swerve": 0.8}, {"pressed": True}, {"trust": 0.6}, {"lane": True}, {"lat": False}, {"v": 31 * MPH},
             {"v": 2 * MPH}, {"probs": (0.35, 0.35), "stds": (1.2, 1.2)}):
    p.y = p.e = p.k = 0.0
    p.turn(desire, 90.0, after=0.0)
    p.run(1.0, probs=kw.get("probs", (0.0, 0.0)))
    p.run(3.0, **kw)
    p.run(4.0)
  p.turn(desire, 30.0)           # a small turn: nothing
  p.run(6.0, probs=(0.35, 0.35))
  p.turn(desire, 90.0, after=1.0)  # the desire lingering after the truck straightened
  p.run(1.0)
  p.frame(other)                   # a new blinker ends the window
  p.run(3.0)
  p.y = 0.6
  p.turn(desire, 90.0)
  p.run(6.0, probs=(0.35, 0.35))   # off-center, lines seen: the centering
  p.y = -0.5
  p.turn(desire, 90.0)
  p.run(1.0)
  p.run(5.0, probs=(0.35, 0.35))   # the blind push, then a line shows the truck


def settle_random_turns(ref, rng, n):
  """Random closed-loop turns: 20-130 deg either way, random speed and model alignment, then random stretches of lines,
  curbs, parked cars, hands, Blendful, lane changes, speed and lateral flicker; Early Right Push toggled per sequence."""
  steps = 0
  for _ in range(n):
    p = SettlePair(ref, rng.random() < 0.5, v=rng.uniform(1.0, 10.0), align=rng.choice([0.05, 0.1, 0.2, 0.4]),
                   y0=rng.uniform(-0.8, 0.8))
    for _ in range(rng.randint(1, 3)):
      p.y, p.e, p.k = rng.uniform(-0.8, 0.8), 0.0, 0.0
      p.turn(rng.choice([log.Desire.turnLeft, log.Desire.turnRight]), rng.uniform(20.0, 130.0), k=rng.uniform(0.05, 0.15),
             after=rng.choice([0.0, 0.0, rng.uniform(0.0, 2.0)]))
      for _ in range(rng.randint(2, 14)):
        kw = {}
        if rng.random() < 0.5:
          kw["probs"] = (rng.choice([0.0, 0.3, 0.5]), rng.choice([0.0, 0.3, 0.5]))
          kw["stds"] = (rng.uniform(0.2, 1.2), rng.uniform(0.2, 1.2))
        if rng.random() < 0.3:
          kw["right_edge"], kw["edge_std"] = rng.uniform(1.5, 4.0), rng.choice([0.3, 0.5, 1.0])
        if rng.random() < 0.15:
          kw["swerve"] = rng.uniform(0.0, 1.2)
        for key, prob in (("pressed", 0.05), ("lane", 0.03)):
          if rng.random() < prob:
            kw[key] = True
        if rng.random() < 0.05:
          kw["lat"] = False
        if rng.random() < 0.1:
          kw["trust"] = rng.uniform(0.0, 1.0)
        if rng.random() < 0.2:
          kw["v"] = rng.uniform(0.0, 15.0)
        if rng.random() < 0.05:
          kw["desire"] = rng.choice([log.Desire.turnLeft, log.Desire.turnRight, log.Desire.laneChangeLeft])
        p.run(rng.uniform(0.1, 1.5), **kw)
    steps += p.steps
  return steps


def settle_constructed(ref):
  n = steps = 0
  for early in (False, True):
    for desire in (log.Desire.turnRight, log.Desire.turnLeft):
      for v, align in ((4.0, 0.2), (2.0, 0.05), (6.7, 0.2), (9.0, 0.05)):
        p = SettlePair(ref, early, v=v, align=align)
        settle_cases(p, desire)
        n += 1
        steps += p.steps
  return n, steps


def settle_carry(ref):
  """The carried turn: the desire cancels at 20-40 deg, Turn Assist carries the rest of a 90 deg turn."""
  res = {"kept on, identical": [0, 0, 0], "late carry": [0, 0, 0], "real signals": [0, 0, 0]}
  late_gap = 0.0
  for early in (False, True):
    for desire in (log.Desire.turnRight, log.Desire.turnLeft):
      for v in (3.0, 4.0, 6.0):
        for cancel in (20.0, 30.0, 40.0):
          for kw in ({"probs": (0.35, 0.35)}, {}, {"right_edge": 2.6, "edge_std": 0.3}):
            p = SettlePair(ref, early, v=v)
            p.turn(desire, 90.0, carry_from=cancel, ref_keeps=True)
            p.run(6.0, **kw)
            res["kept on, identical"][0] += p.steps
            assert p.diffs == 0
            for gap in (1, 2, 3):
              q = SettlePair(ref, early, strict=False, v=v)
              q.turn(desire, 90.0, carry_from=cancel, gap=gap, ref_keeps=True)
              q.run(6.0, **kw)
              res["late carry"][0] += q.steps
              res["late carry"][1] += q.diffs
              res["late carry"][2] += q.out_diffs
              late_gap = max(late_gap, q.max_turned_gap)
            r = SettlePair(ref, early, strict=False, v=v)
            r.turn(desire, 90.0, carry_from=cancel)
            r.run(6.0, **kw)
            res["real signals"][0] += r.steps
            res["real signals"][1] += r.diffs
            res["real signals"][2] += r.out_diffs
            assert r.diffs > 0  # ca08aac9 never settled after a carried turn (cancel under 45 deg); fm-dev-sync does
  return res, late_gap


# ---- 6. Turn Assist through queues ----------------------------------------------------------------------------------

def turn_assist_queues(ref_ta, ref_qg, ref_tp, stride):
  pr12 = _load(Path(__file__).with_name("pr12_turn_carry_equivalence.py"), "pr12_turn_carry_equivalence")
  pr11 = pr12.pr11
  new_ta = pr12.new_mod

  class QueuePair(pr12.CarryPair):
    def step(self, k, direction, v, actual, pressed, lat, model, a_ego, lane_changing, torque, strict=True, hold=False):
      args = (k, direction, v, actual, pressed, lat)
      kw = {"model_v2": model, "a_ego": a_ego, "lane_changing": lane_changing, "steering_torque": torque}
      o, n = self.old.update(*args, **kw), self.new.update(*args, **kw, queue_hold=hold)
      lo, ln = pr11.logged(self.old), pr11.logged(self.new)
      self.steps += 1
      blocked_now = hold and self.old.log_source == ref_ta.SOURCE_CARRY
      self.blocked = getattr(self, "blocked", False) or blocked_now
      if o == n and lo == ln:
        self.blocked = False  # the two have come back together
      else:
        self.diffs += 1
        self.blocked_diffs = getattr(self, "blocked_diffs", 0) + 1
        if not self.blocked:  # a difference not caused by a carry the hold blocked
          diff = {f: (x, y) for f, x, y in zip(pr11.LOG_FIELDS, lo, ln, strict=True) if x != y}
          raise AssertionError(f"turn assist step {self.steps}: {o} vs {n} (hold {hold}, ref source {self.old.log_source})"
                               f" {diff} carry ref {self.old.carry_sign} new {self.new.carry_sign}")
      bump("ta hold-blocked carry steps", int(hold and self.old.log_source == ref_ta.SOURCE_CARRY))
      bump("ta carry steps (new)", int(self.new.log_source == new_ta.SOURCE_CARRY))
      assert not (hold and self.new.log_source == new_ta.SOURCE_CARRY), "a hold carried"
      return n

  n = steps = diffs = 0
  for i, bits in enumerate(range(0, 2 ** len(pr11.TOGGLES), stride)):
    settings = pr11.combo_settings(bits, i)
    for side in (1, -1):
      dp = DHPair(ref_qg, ref_tp, True, True)
      dh_queue_cases(dp, side)
      # the edge ca08aac9 relied on not happening: the hidden desire reaching controlsd with lateral still on
      dp.run(10, v=2.0, d=9.0, k=side * 0.05, left=side < 0, right=side > 0)
      dp.run(1, v=0.29, d=7.0, k=side * 0.08, left=side < 0, right=side > 0)
      dp.run(30, v=0.6, d=6.0, k=side * 0.08, left=side < 0, right=side > 0)
      dp.run(20, v=1.5, d=12.0, k=side * 0.1, left=side < 0, right=side > 0)
      p = QueuePair(ref_ta, settings)
      model = pr11.corner(10.0, start=0.0, side=side)
      lag_v = 0.0
      for v, k, direction, hold in dp.trace:
        for j in range(5):  # 100 Hz controlsd steps per 20 Hz model frame
          lv = lag_v if j == 0 else v  # controlsd's first step still sees the speed before (lateral on above 0.3 m/s)
          p.step(k, direction, lv, k, False, lv > 0.3, model, 0.0, False, 0.0, hold=hold)
        lag_v = v
      assert not p.blocked  # back together by the end of the sequence
      n += 1
      steps += p.steps
      diffs += p.diffs
  return n, steps, diffs


# ---- 7. mutation check ----------------------------------------------------------------------------------------------

def mutation(ref_qg, ref_tp, ref_settle):
  out = {}
  # Queue Guard on in the new DesireHelper only
  p = DHPair(ref_qg, ref_tp, False, False)
  p.new.fm_queue_guard.params.values[new_qg.PARAM] = True
  p.new.fm_queue_guard.read_params()
  try:
    dh_queue_cases(p, 1)
    out["Queue Guard"] = 0
  except AssertionError:
    out["Queue Guard"] = 1
  # Turn Pulse on in the new DesireHelper only
  p = DHPair(ref_qg, ref_tp, False, False)
  p.new.fm_turn_pulse.params.values[new_tp.PARAM] = True
  p.new.fm_turn_pulse.read_params()
  try:
    dh_queue_cases(p, -1)
    out["Turn Pulse"] = 0
  except AssertionError:
    out["Turn Pulse"] = 1
  # Early Right Push on the new side only; Settle off on the new side only
  for name, early_new, enabled_new in (("Early Right Push", True, True), ("Settle After Turn", False, False)):
    a, b, diffs = ref_settle.TurnSettle(), new_settle.TurnSettle(), 0
    rng = np.random.default_rng(5)
    for kw in random_streams(rng, 20000):
      x = a.update(True, **kw, early=False)
      y = b.update(enabled_new, **kw, early=early_new)
      diffs += x != y
    out[name] = diffs
  return out


def main():
  ref_dir = Path(sys.argv[1])
  n_random = int(sys.argv[2]) if len(sys.argv) > 2 else 1500
  stride = int(sys.argv[3]) if len(sys.argv) > 3 else 1
  ref_qg = _load(ref_dir / "fm_queue_guard.py", "fm_queue_guard_ca08aac9")
  ref_tp = _load(ref_dir / "fm_turn_pulse.py", "fm_turn_pulse_ca08aac9")
  ref_settle = _load(ref_dir / "fm_turn_settle.py", "fm_turn_settle_ca08aac9")
  ref_ta = _load(ref_dir / "fm_turn_assist.py", "fm_turn_assist_ca08aac9")
  rng = random.Random(13)
  nrng = np.random.default_rng(13)

  steps = queue_guard_random(ref_qg, rng, n_random)
  print(f"1. Queue Guard vs ca08aac9, random: {n_random} sequences, {steps} steps, identical "
        f"(hold steps {COUNT.get('qg hold steps', 0)})")
  steps = turn_pulse_random(ref_tp, rng, n_random)
  print(f"2. Turn Pulse vs ca08aac9, random: {n_random} sequences, {steps} steps, identical "
        f"(pulses {COUNT.get('tp pulses', 0)})")
  steps = dh_random(ref_qg, ref_tp, rng, n_random)
  cases = csteps = 0
  for qg_on in (False, True):
    for tp_on in (False, True):
      for side in (1, -1):
        p = DHPair(ref_qg, ref_tp, qg_on, tp_on)
        dh_queue_cases(p, side)
        cases += 1
        csteps += p.steps
  print(f"3. DesireHelper vs ca08aac9's order: random {n_random} sequences, {steps} frames; constructed {cases} sequences "
        f"(4 toggle combinations x 2 sides), {csteps} frames; identical (hold frames {COUNT.get('dh hold frames', 0)}, "
        f"model pulses {COUNT.get('dh model pulses', 0)})")
  n_settle = max(n_random // 15, 20)
  steps = settle_random(ref_settle, nrng, n_settle, 3000)
  steps2 = settle_from_model_random(ref_settle, nrng, n_settle, 2000)
  ncase, csteps = settle_constructed(ref_settle)
  acting0 = COUNT.get('closed-loop settle frames', 0)
  steps3 = settle_random_turns(ref_settle, rng, n_random)
  phases = {k[7:]: v for k, v in sorted(COUNT.items()) if k.startswith(("settle phase", "settle limit")) and v}
  print(f"4. Settle After Turn (+ Early Right Push, toggled within runs) vs ca08aac9, no carry: update() random "
        f"{n_settle} x 3000 = {steps} frames, settle_from_model random {n_settle} x 2000 = {steps2} frames, constructed "
        f"closed loop {ncase} sequences {csteps} frames ({acting0} with Settle acting), random closed-loop turns "
        f"{n_random} sequences {steps3} frames ({COUNT.get('closed-loop settle frames', 0) - acting0} acting); "
        f"identical (output, {', '.join(SETTLE_FIELDS)}, state). "
        f"Active frames {COUNT.get('settle active frames', 0)}; nonzero phases/limits {phases}")
  res, late_gap = settle_carry(ref_settle)
  print("5. Settle with a carry (fm-dev-sync change), each case vs ca08aac9: " +
        "; ".join(f"{k}: {s} frames, {d} differing ({o} in the output or a logged field)" for k, (s, d, o) in res.items()) +
        f"; late carry: most degrees not counted {math.degrees(late_gap):.2f}")
  ncase, steps, diffs = turn_assist_queues(ref_ta, ref_qg, ref_tp, stride)
  print(f"6. Turn Assist through queues (Queue Guard on, {len(range(0, 2 ** 10, stride))} Hold/Lead/carry toggle combinations x 2 "
        f"sides): {steps} steps, {diffs} differing, all from a step where ca08aac9 carried a turn out of a hold "
        f"({COUNT.get('ta hold-blocked carry steps', 0)} such steps) until the two agree again; carry steps on the new "
        f"side {COUNT.get('ta carry steps (new)', 0)}, all outside holds")
  out = mutation(ref_qg, ref_tp, ref_settle)
  print(f"7. mutation check (changed on the new side only): {out}")
  assert all(out.values())


if __name__ == "__main__":
  main()
