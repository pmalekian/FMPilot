#!/usr/bin/env python3
"""FM Pilot plan PR 11: same-input equivalence of Turn Lead (with Turn Hold, Smooth Turn Start and the Turn Assist log)
against fm-dev-nav-settle ca08aac9, and of Turn Hold with Turn Lead off against plan PR 10's module.

1. ca08aac9's fm_turn_assist.py (REF, extracted with `git show ca08aac9:<path>`) with its folded Turn Hold toggles ON and
   Turn Boost and Finish Turn After Signal Cancels OFF (dropped / plan PR 12), against the fm-dev-sync module, with the
   same random and constructed inputs and the same Turn Lead and Turn Hold settings (every toggle combination in the
   constructed cases). Compares the request out and every logged field every step, exactly.
2. Plan PR 10's module (PR10, `git show 0340081:<path>`) against the fm-dev-sync module with Turn Lead off and its
   options stored on: Turn Hold-only behavior must be unchanged.
3. Mutation check: one Turn Lead setting changed on the new side only must show up as a difference.

  python tools/fm/tests/pr11_turn_lead_equivalence.py REF PR10 [n_random]
"""
import importlib.util
import random
import sys
from pathlib import Path
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib import fm_turn_assist as new_mod


def _load(path, name):
  spec = importlib.util.spec_from_file_location(name, path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


# the helpers (fake params, corner paths, the folded toggles, log fields) are PR 10's
pr10 = _load(Path(__file__).with_name("pr10_turn_hold_equivalence.py"), "pr10_turn_hold_equivalence")
FakeParams, corner, LOG_FIELDS, FOLDED_ON = pr10.FakeParams, pr10.corner, pr10.LOG_FIELDS, pr10.FOLDED_ON
LEFT, RIGHT, NONE = pr10.LEFT, pr10.RIGHT, pr10.NONE

NOT_PORTED_OFF = {"FM_TurnBoostEnabled": False, "FM_TurnCarryThrough": False, "FM_TurnBoostLeft": 1.1,
                  "FM_TurnBoostRight": 1.2, "FM_TurnCarryTurnInOnly": True}
HOLD_TOGGLES = ("FM_TurnHoldEnabled", "FM_TurnHoldPathSource", "FM_TurnHoldDriverAdds", "FM_TurnHoldFollowExit",
                "FM_TurnRestartAfterStop", "FM_TurnPhaseIn")
LEAD_TOGGLES = ("FM_TurnLeadEnabled", "FM_TurnLeadAdaptive", "FM_TurnLeadGate", "FM_TurnLeadYield")
TOGGLES = HOLD_TOGGLES + LEAD_TOGGLES
# Turn Lead Min Speed (mph), Turn Lead Start Distance (m), Turn Path Tightest Radius (m), incl. out-of-range and bad values
MIN_SPEEDS = (2.0, 3.0, 5.0, 7.0, 1.0, 10.0, None, "x", float("nan"))
GATE_DISTS = (3.0, 5.0, 7.0, 10.0, 1.0, 50.0, None, "x", float("nan"))
RADII = (8.0, 10.0, 12.0, 15.0, 20.0, 5.0, 30.0, None, "x", float("nan"))

COVERAGE: dict[str, int] = {}


def logged(a):
  m = SimpleNamespace()
  a.fill_log(m)
  return tuple(getattr(m, f, 0) for f in LOG_FIELDS)


def tally(log):
  rec = dict(zip(LOG_FIELDS, log, strict=True))
  for f in ("lead", "driverAdds", "holdPath", "holdFloor", "nudge", "smoothCut", "exitFade", "earlyCorrection", "restarts"):
    if rec[f]:
      COVERAGE[f] = COVERAGE.get(f, 0) + 1
  for f in ("releaseReason", "leadYield"):
    if rec[f]:
      key = f"{f}{rec[f]}"
      COVERAGE[key] = COVERAGE.get(key, 0) + 1
  if rec["lead"] and rec["holdFloor"]:
    COVERAGE["lead+holdFloor"] = COVERAGE.get("lead+holdFloor", 0) + 1


class Pair:
  """old: REF (ca08aac9, folded on, Boost/carry off) or PR 10's module; new: the fm-dev-sync module."""

  def __init__(self, old_mod, settings, ref=True, new_settings=None):
    old_values = (NOT_PORTED_OFF | FOLDED_ON | settings) if ref else settings
    self.old = old_mod.FmTurnAssist(FakeParams(old_values))
    self.new_params = FakeParams(settings if new_settings is None else new_settings)
    self.new = new_mod.FmTurnAssist(self.new_params)
    self.steps = 0
    self.diffs = 0

  def step(self, k, direction, v, actual, pressed, lat, model, a_ego, lane_changing, torque, strict=True):
    args = (k, direction, v, actual, pressed, lat)
    kw = {"model_v2": model, "a_ego": a_ego, "lane_changing": lane_changing, "steering_torque": torque}
    o, n = self.old.update(*args, **kw), self.new.update(*args, **kw)
    lo, ln = logged(self.old), logged(self.new)
    self.steps += 1
    tally(ln)
    if o != n or lo != ln:
      self.diffs += 1
      if strict:
        diff = {f: (x, y) for f, x, y in zip(LOG_FIELDS, lo, ln, strict=True) if x != y}
        raise AssertionError(f"step {self.steps}: out {o!r} vs {n!r}, log diffs {diff}, inputs {args} "
                             f"{kw | {'model_v2': None}}")
    return n


def random_settings(rng, lead_on=None):
  s = {key: rng.random() < 0.6 for key in TOGGLES}
  if lead_on is not None:
    s["FM_TurnLeadEnabled"] = lead_on
  elif rng.random() < 0.8:
    s["FM_TurnLeadEnabled"] = True
  s["FM_TurnLeadMinSpeed"] = rng.choice(MIN_SPEEDS)
  s["FM_TurnLeadGateDist"] = rng.choice(GATE_DISTS)
  s["FM_TurnPathMinRadius"] = rng.choice(RADII)
  return s


def random_sequence(old_mod, rng, models, ref=True):
  pair = Pair(old_mod, random_settings(rng, lead_on=None if ref else False), ref=ref)
  if not ref:  # Turn Lead off, its options stored on: what a fm-dev-nav-settle setup might hold
    pair.new_params.values |= {"FM_TurnLeadAdaptive": True, "FM_TurnLeadGate": True, "FM_TurnLeadYield": True}
  # half the sequences stay in Turn Lead's world: 4-17 mph, a corner ahead, a model that turns in late
  focus = rng.random() < 0.5
  targets = (0.0, 0.0, 0.005, 0.01, 0.02, 0.04) if focus else (0.0, 0.0, 0.01, 0.02, 0.06, 0.1, 0.15)
  direction = rng.choice([LEFT, RIGHT]) if focus else rng.choice([LEFT, RIGHT, NONE, None])
  side = 1 if direction == RIGHT else -1  # focus: mostly corners the turn's way
  corners = [m for m in models if side * getattr(getattr(m, "position", None), "y", [0.0])[-1] > 0.0]
  mph = rng.uniform(4, 17) if focus else rng.uniform(0, 22)
  k, actual, pressed, torque, lat = 0.0, 0.0, False, 0.0, True
  model = rng.choice(corners if focus else models)
  a_ego, lane = 0.0, False
  for _ in range(rng.randint(200, 1500)):
    if rng.random() < 0.006:
      direction = rng.choice([LEFT, RIGHT, NONE, None])
    sign = 1 if direction == RIGHT else -1 if direction == LEFT else rng.choice([1, -1])
    if rng.random() < 0.03:
      model = rng.choice(corners if focus and rng.random() < 0.8 else models)
    if rng.random() < 0.01:
      lat = rng.random() < (0.95 if focus else 0.8)
    if rng.random() < 0.004:
      lane = not lane
    if rng.random() < 0.05:
      pressed = rng.random() < (0.15 if focus else 0.3)
      torque = rng.choice([-1, 1]) * rng.choice([rng.uniform(0, 0.5), rng.uniform(0.5, 1.6), rng.uniform(1.4, 3.0)])
    if rng.random() < 0.02:
      pressed = not pressed  # blips
    mph = min(max(mph + rng.gauss(0, 0.15), 0.0), 25.0)
    if focus and rng.random() < 0.01:
      mph = rng.uniform(2, 17)
    if rng.random() < 0.003:
      mph = 0.0
    a_ego = rng.choice([a_ego, a_ego, a_ego, rng.uniform(-3, 1.5)])
    # model request: often late (near 0) so Turn Lead has room, with smooth targets, spikes and let-go
    k += (sign * rng.choice(targets) - k) * 0.05 if rng.random() < 0.3 else rng.gauss(0, 0.003)
    if rng.random() < 0.01:
      k += sign * rng.uniform(0.05, 0.15)  # crawl spike
    follow = 0.02 if focus else 0.08
    actual += (k + (pair.new.log_out - k) * 0.5 - actual) * follow + rng.gauss(0, 0.001)  # the truck follows the request
    pair.step(k, direction, mph * CV.MPH_TO_MS, actual, pressed, lat, model, a_ego, lane, torque)
  return pair


def run(p, k, direction, mph=10, steps=1, actual=None, pressed=False, lat=True, model=None, a_ego=0.0, lane=False,
        torque=0.0, follow=False, strict=True):
  """follow: the truck's curve follows what was asked (0.1 of the way each step), from `actual`."""
  out, act = k, (k if actual is None else actual)
  for _ in range(steps):
    out = p.step(k, direction, mph * CV.MPH_TO_MS, act, pressed, lat, model, a_ego, lane, torque, strict=strict)
    if follow:
      act += (out - act) * 0.1
  return out


def scenarios(p, s, direction, strict=True):
  """Turn Lead's cases, both with and without Turn Hold: approach, wait for the corner, below / above min speed, hands
  yield (paused and off), a tight corner against the radius cap, lead + hold handoff, stop-and-restart."""
  kw = {"strict": strict}
  far, near, tight = corner(12.0, start=6.0, side=s), corner(12.0, start=1.0, side=s), corner(5.0, start=0.0, side=s)
  wide, straight = corner(30.0, start=3.0, side=s), corner(None)
  # approach at 13 mph: straight, the corner far (gate closed), near (open), turning in late, the truck catching up
  run(p, 0.0, direction, mph=13, steps=60, actual=0.0, model=straight, **kw)
  run(p, 0.0, direction, mph=13, steps=40, actual=0.0, model=far, **kw)
  run(p, s * 0.005, direction, mph=12, steps=40, actual=0.0, model=near, **kw)
  run(p, s * 0.01, direction, mph=11, steps=120, actual=0.0, model=near, follow=True, **kw)
  run(p, s * 0.02, direction, mph=11, steps=60, actual=s * 0.07, model=wide, **kw)
  # below / above Turn Lead Min Speed: a sweep from 1 to 17 mph with the corner near
  for mph in (1, 2, 2.5, 3, 4, 5, 6, 7, 8, 9, 12, 13.5, 15, 16, 17):
    run(p, 0.0, direction, mph=mph, steps=15, actual=0.0, model=near, **kw)
  # braking: stopping short of the point it reads, then not
  run(p, 0.0, direction, mph=9, steps=30, actual=0.0, model=near, a_ego=-3.0, **kw)
  run(p, 0.0, direction, mph=9, steps=30, actual=0.0, model=near, a_ego=-0.6, **kw)
  # the model steers away; a lane change; lateral off; no model
  run(p, -s * 0.01, direction, mph=10, steps=20, actual=0.0, model=near, **kw)
  run(p, -s * 0.002, direction, mph=10, steps=20, actual=0.0, model=near, **kw)
  run(p, 0.0, direction, mph=10, steps=20, actual=0.0, model=near, lane=True, **kw)
  run(p, 0.0, direction, mph=10, steps=60, actual=0.0, model=near, lat=False, **kw)
  run(p, 0.0, direction, mph=10, steps=60, actual=0.0, model=None, **kw)
  run(p, 0.0, direction, mph=10, steps=60, actual=0.0, model=SimpleNamespace(), **kw)
  # hands: a press against the turn while leading, before turn-in (pause, let go, back), brief light, firm, into the turn
  run(p, 0.0, direction, mph=10, steps=60, actual=0.0, model=near, **kw)
  run(p, 0.0, direction, mph=10, steps=30, actual=0.0, model=near, pressed=True, torque=s * 1.0, **kw)
  run(p, 0.0, direction, mph=10, steps=70, actual=0.0, model=near, **kw)
  run(p, 0.0, direction, mph=10, steps=10, actual=0.0, model=near, pressed=True, torque=s * 0.8, **kw)
  run(p, 0.0, direction, mph=10, steps=20, actual=0.0, model=near, **kw)
  run(p, 0.0, direction, mph=10, steps=1, actual=0.0, model=near, pressed=True, torque=s * 1.6, **kw)
  run(p, 0.0, direction, mph=10, steps=60, actual=0.0, model=near, pressed=True, torque=-s * 1.0, **kw)
  # a tight 5 m corner: Turn Lead runs into the radius cap; turning past 10 deg, then a press against: off for the turn
  run(p, 0.0, direction, mph=9, steps=80, actual=s * 0.02, model=tight, **kw)
  run(p, s * 0.04, direction, mph=9, steps=60, actual=s * 0.06, model=tight, **kw)
  run(p, s * 0.04, direction, mph=9, steps=40, actual=s * 0.03, model=tight, pressed=True, torque=s * 1.0, **kw)
  run(p, s * 0.02, direction, mph=9, steps=80, actual=s * 0.02, model=tight, **kw)
  # lead + hold handoff: Turn Lead asks, the model lets go and the path straightens, the hold keeps it, signal cancel
  run(p, s * 0.01, direction, mph=10, steps=60, actual=0.0, model=near, follow=True, **kw)
  run(p, 0.0, direction, mph=10, steps=100, actual=s * 0.04, model=straight, **kw)
  run(p, s * 0.01, direction, mph=10, steps=200, actual=s * 0.08, model=wide, **kw)
  run(p, 0.0, NONE, mph=10, steps=60, actual=s * 0.02, model=straight, **kw)


def stop_and_restart(p, s, direction, strict=True):
  """A creep toward the corner with Turn Lead adding, a press that pauses or switches it off, a stop (lateral off), the
  pull-away into the real turn."""
  kw = {"strict": strict}
  near = corner(10.0, start=1.0, side=s)
  run(p, s * 0.10, direction, mph=4, steps=60, actual=s * 0.01, model=near, **kw)
  run(p, 0.0, direction, mph=4, steps=40, actual=s * 0.01, model=near, pressed=True, torque=s * 1.0, **kw)
  run(p, s * 0.03, direction, mph=0.0, steps=300, actual=s * 0.01, model=near, lat=False, **kw)
  run(p, 0.0, direction, mph=4, steps=80, actual=0.0, model=near, **kw)
  # past 10 deg: switched off for the turn, then a stop and a pull-away (no restart past 10 deg)
  run(p, s * 0.02, direction, mph=8, steps=150, actual=s * 0.08, model=near, **kw)
  run(p, 0.0, direction, mph=8, steps=40, actual=s * 0.05, model=near, pressed=True, torque=s * 1.0, **kw)
  run(p, 0.0, direction, mph=0.0, steps=100, actual=s * 0.05, model=near, lat=False, **kw)
  run(p, 0.0, direction, mph=6, steps=100, actual=s * 0.02, model=near, **kw)
  run(p, 0.0, NONE, mph=6, steps=60, actual=0.0, model=near, **kw)


def with_hold(p, s, direction, strict=True):
  """Turn Lead next to Turn Hold's other parts: Turn Nudge at 4 mph with the corner right ahead, Hold Adds Your Steering,
  the model's exit past 45 deg, a full tight turn released at 75 deg, the signal-cancel fade."""
  kw = {"strict": strict}
  m = corner(12.0, start=0.0, side=s)
  run(p, 0.0, direction, mph=4, steps=60, actual=0.0, model=m, **kw)
  run(p, 0.0, direction, mph=4, steps=20, actual=0.0, model=m, pressed=True, torque=-s * 1.0, **kw)
  run(p, 0.0, direction, mph=4, steps=80, actual=0.0, model=m, **kw)
  run(p, s * 0.03, direction, mph=9, steps=60, actual=s * 0.03, model=m, **kw)
  run(p, s * 0.03, direction, mph=9, steps=20, actual=s * 0.19, model=m, pressed=True, torque=-s * 1.5, **kw)
  run(p, s * 0.03, direction, mph=9, steps=20, actual=s * 0.19, model=m, **kw)
  run(p, s * 0.09, direction, mph=10, steps=250, actual=s * 0.09, model=m, follow=True, **kw)  # past 45 deg
  run(p, s * 0.02, direction, mph=10, steps=60, actual=s * 0.05, model=corner(None), **kw)  # the model lets go
  run(p, 0.0, NONE, mph=10, steps=60, actual=s * 0.01, **kw)
  run(p, s * 0.15, direction, mph=10, steps=300, actual=s * 0.15, model=corner(8.0, start=0.0, side=s), **kw)  # 75 deg
  run(p, s * 0.02, direction, mph=10, steps=50, actual=s * 0.05, **kw)
  run(p, s * 0.09, direction, mph=8, steps=60, actual=s * 0.01, model=m, **kw)
  run(p, s * 0.03, NONE, mph=8, steps=60, actual=s * 0.01, model=m, **kw)  # signal cancel: the hold fades


def combo_settings(bits, i):
  s = {key: bool(bits >> j & 1) for j, key in enumerate(TOGGLES)}
  s["FM_TurnLeadMinSpeed"] = MIN_SPEEDS[i % len(MIN_SPEEDS)]
  s["FM_TurnLeadGateDist"] = GATE_DISTS[(i // 3) % len(GATE_DISTS)]
  s["FM_TurnPathMinRadius"] = RADII[(i // 7) % len(RADII)]
  return s


def constructed(ref_mod):
  n = 0
  for i, bits in enumerate(range(2 ** len(TOGGLES))):
    settings = combo_settings(bits, i)
    for direction, s in ((RIGHT, 1), (LEFT, -1)):
      for case in (scenarios, stop_and_restart, with_hold):
        case(Pair(ref_mod, settings), s, direction)
        n += 1
  return n


def mutation_check(ref_mod):
  """One setting changed on the new side only: the comparison must catch it."""
  base = {"FM_TurnLeadEnabled": True, "FM_TurnLeadAdaptive": False, "FM_TurnLeadGate": False, "FM_TurnLeadYield": False,
          "FM_TurnLeadMinSpeed": 7.0, "FM_TurnLeadGateDist": 5.0, "FM_TurnPathMinRadius": 12.0, "FM_TurnHoldEnabled": True}
  changes = {"FM_TurnLeadEnabled": False, "FM_TurnLeadAdaptive": True, "FM_TurnLeadGate": True, "FM_TurnLeadYield": True,
             "FM_TurnLeadMinSpeed": 3.0, "FM_TurnPathMinRadius": 8.0, "FM_TurnHoldEnabled": False, "FM_TurnPhaseIn": True}
  caught = {}
  for key, value in changes.items():
    diffs = 0
    for direction, s in ((RIGHT, 1), (LEFT, -1)):
      for case in (scenarios, stop_and_restart, with_hold):
        p = Pair(ref_mod, base, new_settings=base | {key: value})
        case(p, s, direction, strict=False)
        diffs += p.diffs
    caught[key] = diffs
  # Turn Lead Start Distance only matters with Wait for the Corner on
  diffs = 0
  for direction, s in ((RIGHT, 1), (LEFT, -1)):
    p = Pair(ref_mod, base | {"FM_TurnLeadGate": True}, new_settings=base | {"FM_TurnLeadGate": True, "FM_TurnLeadGateDist": 9.0})
    scenarios(p, s, direction, strict=False)
    diffs += p.diffs
  caught["FM_TurnLeadGateDist"] = diffs
  return caught


def main():
  ref = _load(sys.argv[1], "fm_turn_assist_ca08aac9")
  pr10_mod = _load(sys.argv[2], "fm_turn_assist_pr10")
  n_random = int(sys.argv[3]) if len(sys.argv) > 3 else 3000
  models = [None, SimpleNamespace(), corner(None)] + [corner(r, start=st, side=sd, length=ln)
                                                      for r in (5.0, 8.0, 10.0, 12.0, 20.0, 30.0)
                                                      for st in (0.0, 1.0, 3.0, 6.0, 8.0) for sd in (1, -1) for ln in (6, 10, 60)]
  rng = random.Random(11)
  steps, read_folded = 0, set()
  for _ in range(n_random):
    pair = random_sequence(ref, rng, models)
    steps += pair.steps
    read_folded |= pair.new_params.reads & set(new_mod.FOLDED_KEYS)
  print(f"1a. vs ca08aac9, random: {n_random} sequences, {steps} steps, identical; steps where each part acted: "
        f"{dict(sorted(COVERAGE.items()))}")
  COVERAGE.clear()
  cases = constructed(ref)
  print(f"1b. vs ca08aac9, constructed: {cases} sequences (3 scenarios x 2 sides x {2 ** len(TOGGLES)} toggle "
        f"combinations; min speed / start distance / radius cycled incl. out-of-range and bad values), identical; "
        f"steps where each part acted: {dict(sorted(COVERAGE.items()))}")
  assert not read_folded, read_folded
  print("    folded keys read by the new module: none")
  COVERAGE.clear()
  steps = 0
  for _ in range(max(n_random // 3, 1)):
    steps += random_sequence(pr10_mod, rng, models, ref=False).steps
  for bits in range(2 ** len(HOLD_TOGGLES)):
    settings = {key: bool(bits >> j & 1) for j, key in enumerate(HOLD_TOGGLES)}
    for direction, s in ((RIGHT, 1), (LEFT, -1)):
      for case in (scenarios, stop_and_restart, with_hold):
        p = Pair(pr10_mod, settings, ref=False,
                 new_settings=settings | {"FM_TurnLeadAdaptive": True, "FM_TurnLeadGate": True, "FM_TurnLeadYield": True,
                                          "FM_TurnLeadMinSpeed": 2.0, "FM_TurnLeadGateDist": 3.0})
        case(p, s, direction)
        steps += p.steps
  print(f"2.  Turn Lead off (options on) vs PR 10's module: {steps} steps (random + {2 ** len(HOLD_TOGGLES)} Turn Hold "
        f"combinations x 3 scenarios x 2 sides), identical; steps where each part acted: {dict(sorted(COVERAGE.items()))}")
  caught = mutation_check(ref)
  print(f"3.  mutation check, differing steps per setting changed on the new side only: {caught}")
  assert all(caught.values()), caught
  print("    every changed setting caught")


if __name__ == "__main__":
  main()
