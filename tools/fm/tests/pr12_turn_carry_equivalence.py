#!/usr/bin/env python3
"""FM Pilot plan PR 12: same-input equivalence of Finish Turn After Signal Cancels (with Turn Lead, Turn Hold, Smooth Turn
Start and the Turn Assist log) against fm-dev-nav-settle ca08aac9.

On PR 11's helpers (tools/fm/tests/pr11_turn_lead_equivalence.py): ca08aac9's fm_turn_assist.py (REF, extracted with
`git show ca08aac9:<path>`) with its folded Turn Hold toggles ON, Turn Boost OFF, FM_TurnCarryThrough ON and Carry Only
at Turn-In (FM_TurnCarryTurnInOnly) ON, against the fm-dev-sync module with FM_TurnCarryThrough ON (Carry Only at
Turn-In folded), with the same random and constructed inputs. Compares the request out and every logged field every
step, exactly. PR 11's script covers the carry off.

  python tools/fm/tests/pr12_turn_carry_equivalence.py REF [n_random] [combo_stride]
"""
import importlib.util
import random
import sys
from pathlib import Path
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib import fm_turn_assist as new_mod


def _load(path, name):
  spec = importlib.util.spec_from_file_location(name, path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


pr11 = _load(Path(__file__).with_name("pr11_turn_lead_equivalence.py"), "pr11_turn_lead_equivalence")
Pair, corner, run, RIGHT, LEFT, NONE = pr11.Pair, pr11.corner, pr11.run, pr11.RIGHT, pr11.LEFT, pr11.NONE
assert pr11.NOT_PORTED_OFF["FM_TurnCarryTurnInOnly"] is True  # the folded behavior on the REF side
CARRY_ON = {"FM_TurnCarryThrough": True}
CARRY_STEPS = {"carry": 0}


class CarryPair(Pair):
  def __init__(self, old_mod, settings, **kw):
    super().__init__(old_mod, settings | CARRY_ON, **kw)

  def step(self, *args, **kw):
    out = super().step(*args, **kw)
    if self.new.log_source == new_mod.SOURCE_CARRY:
      CARRY_STEPS["carry"] += 1
    return out


def carry_cases(p, s, direction, strict=True):
  """Blinker off at turn-in with the truck turning (carry), and every way it ends: straightening, 6 s, a light and a
  firm press against, a press into the turn, lateral off, a lane change, 20 mph, a new desire either side; and cancels
  past 45 deg or while unwinding (no carry)."""
  kw = {"strict": strict}
  m, straight = corner(10.0, start=0.0, side=s), corner(None)
  starts = ((0.09, 0.001), (0.02, 0.02), (0.0, 0.012))
  ends = ({"actual": s * 0.005}, {"steps": 700, "mph": 3}, {"pressed": True, "torque": s * 0.3, "steps": 30},
          {"pressed": True, "torque": s * 0.6}, {"pressed": True, "torque": -s * 1.5, "steps": 40}, {"lat": False},
          {"lane": True}, {"mph": 21}, {"direction": RIGHT}, {"direction": LEFT}, {"actual": s * 0.0095, "steps": 20})
  for k0, act0 in starts:
    for end in ends:
      run(p, s * k0, direction, mph=5, steps=60, actual=s * act0, model=m, **kw)
      run(p, s * 0.01, NONE, mph=5, steps=10, actual=s * 0.08, model=m, **kw)
      e = dict(end)
      d = e.pop("direction", NONE)
      run(p, s * 0.01, d, mph=e.pop("mph", 5), steps=e.pop("steps", 3), actual=e.pop("actual", s * 0.08), model=m,
          **e, **kw)
      run(p, 0.0, NONE, mph=5, steps=80, actual=0.0, model=straight, **kw)
  # late cancel (past 45 deg) and a cancel while unwinding: no carry, the usual fade
  run(p, s * 0.1, direction, mph=10, steps=200, actual=s * 0.1, model=m, **kw)
  run(p, s * 0.01, NONE, mph=10, steps=60, actual=s * 0.1, model=m, **kw)
  run(p, s * 0.09, direction, mph=10, steps=50, actual=s * 0.1, model=m, **kw)
  run(p, s * 0.05, direction, mph=10, steps=5, actual=s * 0.05, model=m, **kw)
  run(p, s * 0.01, NONE, mph=10, steps=60, actual=s * 0.05, model=m, **kw)
  # just under the limits: carries
  run(p, s * 0.09, direction, mph=10, steps=100, actual=s * 0.1, model=m, **kw)
  run(p, s * 0.075, direction, mph=10, steps=2, actual=s * 0.075, model=m, **kw)
  run(p, s * 0.01, NONE, mph=10, steps=100, actual=s * 0.075, model=m, follow=True, **kw)


def main():
  ref = _load(sys.argv[1], "fm_turn_assist_ca08aac9")
  n_random = int(sys.argv[2]) if len(sys.argv) > 2 else 1500
  stride = int(sys.argv[3]) if len(sys.argv) > 3 else 1
  models = [None, SimpleNamespace(), corner(None)] + [corner(r, start=st, side=sd, length=ln)
                                                      for r in (5.0, 8.0, 10.0, 12.0, 20.0, 30.0)
                                                      for st in (0.0, 1.0, 3.0, 6.0, 8.0) for sd in (1, -1) for ln in (6, 10, 60)]
  pr11.Pair = CarryPair  # random_sequence builds its pair through the module global
  rng = random.Random(12)
  steps = 0
  for _ in range(n_random):
    steps += pr11.random_sequence(ref, rng, models).steps
  print(f"1. vs ca08aac9 (carry on, Carry Only at Turn-In on), random: {n_random} sequences, {steps} steps, identical; "
        f"carried steps: {CARRY_STEPS['carry']}")
  CARRY_STEPS["carry"] = 0
  n = steps = 0
  for i, bits in enumerate(range(0, 2 ** len(pr11.TOGGLES), stride)):
    settings = pr11.combo_settings(bits, i)
    for direction, s in ((RIGHT, 1), (LEFT, -1)):
      for case in (carry_cases, pr11.scenarios, pr11.with_hold):
        p = CarryPair(ref, settings)
        case(p, s, direction)
        n += 1
        steps += p.steps
  print(f"2. vs ca08aac9 (carry on), constructed: {n} sequences ({2 ** len(pr11.TOGGLES) // stride} toggle combinations "
        f"x 3 cases x 2 sides), {steps} steps, identical; carried steps: {CARRY_STEPS['carry']}")
  # mutation check: carry off on the new side only must show up
  diffs = 0
  for direction, s in ((RIGHT, 1), (LEFT, -1)):
    p = CarryPair(ref, {"FM_TurnHoldEnabled": True}, new_settings={"FM_TurnHoldEnabled": True})
    carry_cases(p, s, direction, strict=False)
    diffs += p.diffs
  print(f"3. mutation check, carry off on the new side only: {diffs} differing steps")
  assert diffs


if __name__ == "__main__":
  main()
