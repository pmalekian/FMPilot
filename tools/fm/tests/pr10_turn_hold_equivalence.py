#!/usr/bin/env python3
"""FM Pilot plan PR 10: same-input equivalence of Turn Hold / Smooth Turn Start / the Turn Assist log against
fm-dev-nav-settle ca08aac9.

Drives the ca08aac9 fm_turn_assist.py (extracted with `git show ca08aac9:<path>`, passed as REF) with its folded toggles
ON (Turn Nudge, Hold Through Early Correction, Ignore Brief Hand Blips, Hold Starts at Turn-In, Hold Only Counts While
Steering, Hold Ignores Crawl Spikes) and Turn Boost, Turn Lead and Finish Turn After Signal Cancels OFF (dropped / later
PRs), and the fm-dev-sync module, with the same random and constructed inputs and the same remaining settings. Compares
the request out and every logged field every step, exactly.

  python tools/fm/tests/pr10_turn_hold_equivalence.py /path/to/ca08aac9/fm_turn_assist.py [n_random]
"""
import importlib.util
import math
import random
import sys
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib import fm_turn_assist as new_mod

LEFT, RIGHT, NONE = new_mod.TurnDirection.turnLeft, new_mod.TurnDirection.turnRight, new_mod.TurnDirection.none
FOLDED_ON = {"FM_TurnNudgeEnabled": True, "FM_TurnHoldEarlyCorrection": True, "FM_TurnHoldReleaseDebounce": True,
             "FM_TurnHoldStartsAtTurnIn": True, "FM_TurnHoldSteeringOnly": True, "FM_TurnHoldSteadyPeak": True}
NOT_PORTED_OFF = {"FM_TurnBoostEnabled": False, "FM_TurnLeadEnabled": False, "FM_TurnCarryThrough": False,
                  "FM_TurnBoostLeft": 1.1, "FM_TurnBoostRight": 1.2, "FM_TurnLeadMinSpeed": 7.0,
                  "FM_TurnLeadAdaptive": True, "FM_TurnLeadGate": True, "FM_TurnLeadGateDist": 5.0,
                  "FM_TurnCarryTurnInOnly": True, "FM_TurnLeadYield": False}
SETTINGS = ("FM_TurnHoldEnabled", "FM_TurnHoldPathSource", "FM_TurnHoldDriverAdds", "FM_TurnHoldFollowExit",
            "FM_TurnRestartAfterStop", "FM_TurnPhaseIn")
LOG_FIELDS = ("sign", "source", "amount", "requestIn", "requestOut", "boost", "lead", "driverAdds", "holdPath", "holdFloor",
              "nudge", "smoothCut", "exitFade", "holdActive", "holdPeak", "holdFloorLevel", "turned", "holdTurned",
              "holdTime", "releaseReason", "earlyCorrection", "againstTime", "leadYield", "restarts")


class FakeParams:
  def __init__(self, values):
    self.values = dict(values)
    self.reads = set()

  def get_bool(self, key):
    self.reads.add(key)
    return bool(self.values.get(key))

  def get(self, key, return_default=False):
    self.reads.add(key)
    return self.values.get(key)


def load_ref(path):
  spec = importlib.util.spec_from_file_location("fm_turn_assist_ca08aac9", path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def corner(radius=None, start=2.0, side=1, length=60):
  xs, ys = [], []
  for i in range(length + 1):
    s = float(i)
    if radius is None or s <= start:
      x, y = s, 0.0
    elif s <= start + radius * math.pi / 2:
      th = (s - start) / radius
      x, y = start + radius * math.sin(th), side * radius * (1.0 - math.cos(th))
    else:
      x, y = start + radius, side * (radius + s - start - radius * math.pi / 2)
    xs.append(x)
    ys.append(y)
  return SimpleNamespace(position=SimpleNamespace(x=xs, y=ys))


def logged(a):
  m = SimpleNamespace()
  a.fill_log(m)
  return tuple(getattr(m, f, 0) for f in LOG_FIELDS)


COVERAGE: dict[str, int] = {}


def tally(log):
  rec = dict(zip(LOG_FIELDS, log, strict=True))
  for f in ("driverAdds", "holdPath", "holdFloor", "nudge", "smoothCut", "exitFade", "earlyCorrection", "restarts"):
    if rec[f]:
      COVERAGE[f] = COVERAGE.get(f, 0) + 1
  if rec["releaseReason"]:
    key = f"release{rec['releaseReason']}"
    COVERAGE[key] = COVERAGE.get(key, 0) + 1


class Pair:
  def __init__(self, ref_mod, settings):
    self.old = ref_mod.FmTurnAssist(FakeParams(NOT_PORTED_OFF | FOLDED_ON | settings))
    self.new_params = FakeParams(settings)
    self.new = new_mod.FmTurnAssist(self.new_params)
    self.steps = 0

  def step(self, k, direction, v, actual, pressed, lat, model, a_ego, lane_changing, torque):
    args = (k, direction, v, actual, pressed, lat)
    kw = {"model_v2": model, "a_ego": a_ego, "lane_changing": lane_changing, "steering_torque": torque}
    o, n = self.old.update(*args, **kw), self.new.update(*args, **kw)
    lo, ln = logged(self.old), logged(self.new)
    self.steps += 1
    tally(ln)
    if o != n or lo != ln:
      diff = {f: (x, y) for f, x, y in zip(LOG_FIELDS, lo, ln, strict=True) if x != y}
      raise AssertionError(f"step {self.steps}: out {o!r} vs {n!r}, log diffs {diff}, inputs {args} {kw | {'model_v2': None}}")
    return n


def random_settings(rng):
  s = {key: rng.random() < 0.6 for key in SETTINGS}
  if rng.random() < 0.85:
    s["FM_TurnHoldEnabled"] = True
  s["FM_TurnPathMinRadius"] = rng.choice([8.0, 10.0, 12.0, 15.0, 20.0, 5.0, 30.0, None, "x", float("nan")])
  return s


def random_sequence(ref_mod, rng, models):
  pair = Pair(ref_mod, random_settings(rng))
  direction = rng.choice([LEFT, RIGHT, NONE, None])
  mph, k, actual, pressed, torque, lat, model = rng.uniform(0, 22), 0.0, 0.0, False, 0.0, True, rng.choice(models)
  a_ego, lane = 0.0, False
  for _ in range(rng.randint(200, 1500)):
    r = rng.random()
    if r < 0.006:
      direction = rng.choice([LEFT, RIGHT, NONE, None])
    sign = 1 if direction == RIGHT else -1 if direction == LEFT else rng.choice([1, -1])
    if rng.random() < 0.03:
      model = rng.choice(models)
    if rng.random() < 0.01:
      lat = rng.random() < 0.8
    if rng.random() < 0.004:
      lane = not lane
    if rng.random() < 0.05:
      pressed = rng.random() < 0.3
      torque = rng.choice([-1, 1]) * rng.choice([rng.uniform(0, 0.5), rng.uniform(0.5, 1.6), rng.uniform(1.4, 3.0)])
    if rng.random() < 0.02:
      pressed = not pressed  # blips
    mph = min(max(mph + rng.gauss(0, 0.15), 0.0), 25.0)
    if rng.random() < 0.003:
      mph = 0.0
    a_ego = rng.choice([a_ego, a_ego, a_ego, rng.uniform(-3, 1.5)])
    # model request: a smooth target toward the turn with spikes and let-go
    k += (sign * rng.choice([0.0, 0.02, 0.06, 0.1, 0.15]) - k) * 0.05 if rng.random() < 0.3 else rng.gauss(0, 0.003)
    if rng.random() < 0.01:
      k += sign * rng.uniform(0.05, 0.15)  # crawl spike
    actual += (k - actual) * 0.08 + rng.gauss(0, 0.001)
    v = mph * CV.MPH_TO_MS
    pair.step(k, direction, v, actual, pressed, lat, model, a_ego, lane, torque)
  return pair


def run(a_pair, k, direction, mph=10, steps=1, actual=None, pressed=False, lat=True, model=None, a_ego=0.0, lane=False,
        torque=0.0):
  out = k
  for _ in range(steps):
    out = a_pair.step(k, direction, mph * CV.MPH_TO_MS, k if actual is None else actual, pressed, lat, model, a_ego, lane,
                      torque)
  return out


def constructed(ref_mod):
  """Turn-in, hand blips, crawl spikes, early correction, model exit, signal cancel, stop-and-restart, nudge, Smooth Turn
  Start, Hold Path Source and Hold Adds Your Steering, both sides, each setting combination."""
  n = 0
  for bits in range(2 ** (len(SETTINGS) - 1)):
    settings = {key: bool(bits >> i & 1) for i, key in enumerate(SETTINGS[1:])} | {"FM_TurnHoldEnabled": True}
    for direction, s in ((RIGHT, 1), (LEFT, -1)):
      # turn-in from a stop with the path, then hold, model eases, exit past 45 deg
      p = Pair(ref_mod, settings)
      m = corner(10.0, start=0.0, side=s)
      run(p, 0.0, direction, mph=0, steps=200, actual=0.0, model=m, lat=False)
      run(p, 0.0, direction, mph=1, steps=60, actual=0.0, model=m)
      run(p, s * 0.09, direction, mph=4, steps=80, actual=s * 0.05, model=m)
      run(p, s * 0.03, direction, mph=6, steps=150, actual=s * 0.09, model=m)
      run(p, s * 0.01, direction, mph=8, steps=200, actual=s * 0.05, model=m)
      run(p, 0.0, NONE, mph=9, steps=80, actual=s * 0.02)
      # hand blips: one frame, short, firm, long, at turn-in and later
      p = Pair(ref_mod, settings)
      run(p, s * 0.09, direction, mph=12.7, steps=60, actual=s * 0.001)
      run(p, s * 0.03, direction, mph=12.7, actual=s * 0.005, pressed=True, torque=s * 0.14)
      run(p, s * 0.03, direction, mph=12.7, steps=10, actual=s * 0.005)
      for torque, steps in ((0.62, 5), (1.01, 1), (2.0, 100), (1.0, 160)):
        run(p, s * 0.03, direction, mph=15, steps=steps, actual=s * 0.001, pressed=True, torque=s * torque)
        run(p, s * 0.03, direction, mph=15, steps=5, actual=s * 0.001)
      p = Pair(ref_mod, settings)
      run(p, s * 0.1, direction, steps=50, actual=s * 0.1)  # past 10 deg
      run(p, s * 0.03, direction, steps=3, actual=s * 0.1, pressed=True, torque=s * 1.0)
      run(p, s * 0.03, direction, steps=30, actual=s * 0.1, pressed=True, torque=s * 0.6)
      run(p, s * 0.03, direction, steps=3, actual=s * 0.1, pressed=True, torque=s * 1.6)
      # crawl spike under 5 mph, then a steady crawl request, then above 5 mph
      p = Pair(ref_mod, settings)
      run(p, s * 0.03, direction, mph=3, steps=50)
      run(p, s * 0.15, direction, mph=3, steps=5)
      run(p, s * 0.04, direction, mph=3, steps=50)
      run(p, s * 0.12, direction, mph=3, steps=40)
      run(p, s * 0.15, direction, mph=7, steps=5)
      run(p, s * 0.02, direction, mph=7, steps=50)
      # early correction: straightening at the start, then late in the turn, then into the turn
      p = Pair(ref_mod, settings)
      run(p, s * 0.09, direction, mph=3, steps=60, actual=0.0)
      run(p, 0.0, direction, mph=3, steps=50, pressed=True, torque=s * 1.5, actual=-s * 0.05)
      run(p, s * 0.03, direction, mph=3, steps=10, actual=0.0)
      run(p, s * 0.1, direction, mph=3, steps=600, actual=s * 0.1)
      run(p, s * 0.05, direction, mph=3, steps=5, pressed=True, torque=s * 1.5, actual=s * 0.05)
      run(p, s * 0.05, direction, mph=3, steps=50, pressed=True, torque=-s * 1.5, actual=s * 0.12)
      # Hold Adds Your Steering and the signal-cancel fade while pressing into the turn
      p = Pair(ref_mod, settings)
      run(p, s * 0.03, direction, steps=60, actual=s * 0.03)
      run(p, s * 0.03, direction, steps=20, pressed=True, torque=-s * 1.5, actual=s * 0.19)
      run(p, s * 0.03, direction, steps=20, actual=s * 0.19)
      run(p, s * 0.03, NONE, steps=5, pressed=True, torque=-s * 1.5, actual=s * 0.1)
      run(p, s * 0.03, NONE, steps=60, actual=s * 0.1)
      # stop before turning (queue), creep, stop, creep
      p = Pair(ref_mod, settings)
      run(p, s * 0.10, direction, mph=2, steps=60, actual=s * 0.03)
      run(p, s * 0.03, direction, mph=0.0, steps=300, actual=s * 0.03, lat=False)
      run(p, s * 0.007, direction, mph=2, steps=60, actual=s * 0.03)
      run(p, s * 0.10, direction, mph=2, steps=40, actual=s * 0.03, pressed=True, torque=s * 1.0)
      run(p, s * 0.03, direction, mph=0.0, steps=100, actual=s * 0.03, lat=False)
      run(p, s * 0.03, direction, mph=3, steps=100, actual=s * 0.05)
      # Turn Nudge: a deliberate nudge into the turn at 4 mph with the corner right ahead, then braking
      p = Pair(ref_mod, settings)
      m = corner(12.0, start=0.0, side=s)
      run(p, 0.0, direction, mph=4, steps=60, actual=0.0, model=m)
      run(p, 0.0, direction, mph=4, steps=20, actual=0.0, model=m, pressed=True, torque=-s * 1.0)
      run(p, 0.0, direction, mph=4, steps=80, actual=0.0, model=m)
      run(p, 0.0, direction, mph=4, steps=20, actual=0.0, model=m, pressed=True, torque=-s * 1.0, a_ego=-1.0)
      run(p, 0.0, direction, mph=4, steps=20, actual=0.0, model=m, pressed=True, torque=s * 1.0)
      # Smooth Turn Start: a stepped request, a drop, a lane change, lateral off, 20 mph
      p = Pair(ref_mod, settings)
      run(p, 0.0, direction, steps=60)
      run(p, s * 0.06, direction, steps=20)
      run(p, 0.0, direction, steps=30)
      run(p, s * 0.06, direction, steps=20, lane=True)
      run(p, s * 0.06, direction, steps=20, lat=False)
      run(p, s * 0.06, direction, mph=21, steps=80)
      run(p, s * 0.06, direction, mph=14, steps=80)
      run(p, s * 0.06, NONE if s > 0 else RIGHT, steps=80)
      # a long gentle turn: the 8 s run out (after turn-in), then the cancel
      p = Pair(ref_mod, settings)
      run(p, s * 0.05, direction, mph=9, steps=1700, actual=s * 0.004)
      run(p, 0.0, NONE, mph=9, steps=60, actual=s * 0.003)
      # a full tight turn: released at 75 deg, then the cancel
      p = Pair(ref_mod, settings)
      run(p, s * 0.15, direction, mph=10, steps=300, actual=s * 0.15)
      run(p, s * 0.02, direction, mph=10, steps=50, actual=s * 0.05)
      run(p, 0.0, NONE, mph=10, steps=60, actual=s * 0.01)
      n += 11
  return n


def main():
  ref = load_ref(sys.argv[1])
  n_random = int(sys.argv[2]) if len(sys.argv) > 2 else 3000
  models = [None, SimpleNamespace(), corner(None)] + [corner(r, start=st, side=sd, length=ln)
                                                      for r in (6.0, 8.0, 10.0, 12.0, 20.0, 30.0)
                                                      for st in (0.0, 1.0, 3.0, 8.0) for sd in (1, -1) for ln in (6, 10, 60)]
  rng = random.Random(10)
  steps = 0
  read_folded = set()
  for _ in range(n_random):
    pair = random_sequence(ref, rng, models)
    steps += pair.steps
    read_folded |= pair.new_params.reads & set(new_mod.FOLDED_KEYS)
  print(f"random: {n_random} sequences, {steps} steps, identical; steps where each part acted: {dict(sorted(COVERAGE.items()))}")
  COVERAGE.clear()
  cases = constructed(ref)
  print(f"constructed: {cases} sequences (11 scenarios x 2 sides x {2 ** (len(SETTINGS) - 1)} setting combinations, Turn Hold on), identical; "
        f"steps where each part acted: {dict(sorted(COVERAGE.items()))}")
  assert not read_folded, read_folded
  print("folded keys read by the new module: none")


if __name__ == "__main__":
  main()
