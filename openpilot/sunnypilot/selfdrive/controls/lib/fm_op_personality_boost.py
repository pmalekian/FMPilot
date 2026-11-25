"""
FM Pilot: OP Personality boost, mode 5 of the Accel Boost picker (FM_AccelBoostMode, fm_accel_boost_mode.py).

Ported from openpilot master (commaai/openpilot, MIT, Harald Schäfer):
- #39430 "Restore acceleration boost for Aggressive personality", merged Nov 19 2026 (ff300ecf): +0.1 m/s^2 on the
  model's acceleration in Aggressive, faded in from 5 to 10 m/s and out as the model brakes from -0.5 to -1.0 m/s^2.
- #39440 "Tune acceleration boost and following distance by personality", merged Nov 23 2026 (6daa874b): the boost
  comes from a personality table, Relaxed 0.0 / Standard 0.05 / Aggressive 0.15 m/s^2, same fades.
openpilot adds it in longitudinal_planner.py right after reading modelV2.action.desiredAcceleration (plus its curve
offset, which sunnypilot does not have). Not in sunnypilot yet.

FM Pilot changes: it is one mode of the picker, so it never stacks with another boost; personality read through
personality_index() (a capnp enum or a plain int; anything else counts as Standard). #39440's other change, Relaxed's
follow time 1.75 -> 1.45 s, is not ported: it is not a boost, and the lead MPC stays sunnypilot's.
"""
import numpy as np

from openpilot.cereal import log

Personality = log.LongitudinalPersonality

ACCEL_BOOST = {  # m/s^2, openpilot #39440
  Personality.relaxed: 0.0,
  Personality.standard: 0.05,
  Personality.aggressive: 0.15,
}
SPEED_BP = [5.0, 10.0]  # m/s: none at 5, full at 10 (#39430)
ACCEL_BP = [-1.0, -0.5]  # m/s^2 of the model's target: none at -1.0, full from -0.5 up (#39430)


def personality_index(personality) -> int:
  """log.LongitudinalPersonality value (0 aggressive, 1 standard, 2 relaxed); anything else counts as standard."""
  try:
    idx = int(getattr(personality, 'raw', personality))
  except (TypeError, ValueError):
    return int(Personality.standard)
  return idx if idx in ACCEL_BOOST else int(Personality.standard)


def get_max_boost(personality) -> float:
  return ACCEL_BOOST[personality_index(personality)]


class OpPersonalityBoost:
  """Adds the personality's boost to the model's (e2e) acceleration target, faded at low speed and when braking."""
  def __init__(self):
    self.total_boost = 0.0

  def update(self, sm, output_a_target_e2e: float) -> float:
    v_ego = sm['carState'].vEgo
    speed_scale = float(np.interp(v_ego, SPEED_BP, [0.0, 1.0]))
    accel_scale = float(np.interp(output_a_target_e2e, ACCEL_BP, [0.0, 1.0]))
    self.total_boost = get_max_boost(sm['selfdriveState'].personality) * speed_scale * accel_scale
    return output_a_target_e2e + self.total_boost
