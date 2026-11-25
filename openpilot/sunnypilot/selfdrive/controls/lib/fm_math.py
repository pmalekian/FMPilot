"""FM Pilot: small scalar helpers for code that runs on every 10 ms control step.

numpy's scalar calls cost microseconds each (np.clip about 3 us, np.interp about 1.6 us on a PC, several times that on
the comma), and FM Pilot's hooks made about a dozen of them per step even with nothing to do.
"""


def step_toward(value: float, target: float, step: float) -> float:
  """value moved toward target by at most step. Same result as float(np.clip(target, value - step, value + step))."""
  return value if value == target else min(max(target, value - step), value + step)
