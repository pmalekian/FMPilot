"""
FM Pilot: Set Speed Assist, ported from EVPilot (evpilotgo/openpilot, branch dev, by Gerry / gerrylum; MIT like openpilot),
openpilot/sunnypilot/selfdrive/controls/lib/set_speed_assist.py at c5943813 (Nov 10 2026), which includes its commits
3241411..c594381 (Oct 30 - Nov 10 2026). The numbers and the boost / grace logic are unchanged.

FM Pilot changes:
- no SetSpeedAssist param: it is mode 3 of FM_AccelBoostMode (fm_boost_mode.py), so it never stacks with Gas or Predictive
- a fresh instance (mode switched on while driving) takes the current engaged state on its first frame instead of
  treating it as a new engage, so switching modes onroad doesn't start an engage grace
- personality read through personality_index(), which takes a capnp enum or a plain int
"""
import numpy as np

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

MAX_BOOST = (0.7, 0.5, 0.3)  # m/s^2 above the model, by driving personality: aggressive, standard, relaxed
MIN_SPEED, LEAD_MIN_SPEED = 20 * CV.MPH_TO_MS, 10 * CV.MPH_TO_MS  # a braking lead can take us below MIN_SPEED, see below
LAT_ACCEL_FADE, MAX_LAT_ACCEL = 0.5, 1.0  # m/s^2, the cap shrinks to 0 through a turn, off above
MODEL_SLOWING, MODEL_BRAKING = -0.1, -0.5  # m/s^2, the cap shrinks from full at a -0.1 model slow-down to 0 at -0.5, braking drops it at once
RAMP_UP, RAMP_DOWN = 0.5, 1.5  # m/s^3
GRACE, GRACE_MARGIN, GRACE_JERK = 3., 0.5, 1.2  # s, m/s^2 of extra model braking it rides through, m/s^3, see below


def personality_index(personality) -> int:
  """0 aggressive, 1 standard, 2 relaxed (log.LongitudinalPersonality order); anything else counts as standard."""
  try:
    idx = int(getattr(personality, 'raw', personality))
  except (TypeError, ValueError):
    return 1
  return idx if 0 <= idx < len(MAX_BOOST) else 1


def get_max_boost(personality) -> float:
  return MAX_BOOST[personality_index(personality)]


class SetSpeedAssist:
  """Raises only the model's accel toward the cruise ramp, which tapers to 0 at the set speed. Lead MPC and cruise stay in the min(), model braking wins."""
  def __init__(self):
    self.boost = 0.
    self.engaged, self.lead, self.grace, self.floor, self.ramp, self.v_min = None, False, 0., 0., 0., MIN_SPEED

  @property
  def total_boost(self) -> float:
    return self.boost

  def update(self, sm, a_e2e: float, a_cruise: float) -> float:
    cs, action, engaged, lead = sm['carState'], sm['modelV2'].action, sm['selfdriveState'].enabled, sm['radarState'].leadOne.present
    v, a_ego = cs.vEgo, cs.aEgo
    lat_accel = abs(action.desiredCurvature) * v ** 2
    ok = engaged and not action.shouldStop and lat_accel < MAX_LAT_ACCEL
    allowed = ok and v > MIN_SPEED
    if self.engaged is None:  # FM Pilot: first frame of a fresh instance, not an engage
      self.engaged = engaged

    # Grace: the model carries a slowdown in progress at an engage, or when a lead we braked for leaves, forward and deepens it
    # at first. For a few seconds a ramp from the truck's accel up to the cruise ramp leads instead, while the model brakes no
    # harder than the truck was then plus a margin; harder braking or a stop hands back at once.
    lead_left = self.lead and not lead and a_ego < MODEL_SLOWING
    if (engaged and not self.engaged) or lead_left:
      ramp = max(a_ego, a_e2e + self.boost) if lead_left else a_ego  # a lead leaving mid-boost never steps the accel down
      self.grace, self.floor, self.ramp, self.v_min = GRACE, min(a_e2e, a_ego) - GRACE_MARGIN, ramp, LEAD_MIN_SPEED if lead_left else MIN_SPEED
    self.engaged, self.lead, self.grace = engaged, lead, self.grace - DT_MDL if ok and v > self.v_min and a_e2e >= self.floor else 0.
    if self.grace > 0 and a_e2e < MODEL_SLOWING:  # carried as a boost, so it hands back to the model smoothly once the braking fades out
      self.ramp = min(self.ramp + GRACE_JERK * DT_MDL, a_cruise)
      self.boost = max(self.ramp - a_e2e, 0.)
      return a_e2e + self.boost

    if a_e2e < MODEL_BRAKING or action.shouldStop:
      self.boost = 0.
    else:
      # Proportional, not cutoffs: the model slows a little as the boost speeds it past what it wants, and hard gates there and in turns pulsed
      cap = get_max_boost(sm['selfdriveState'].personality) * float(np.interp(a_e2e, [MODEL_BRAKING, MODEL_SLOWING], [0., 1.]) *
                                                                    np.interp(lat_accel, [LAT_ACCEL_FADE, MAX_LAT_ACCEL], [1., 0.]))
      target = float(np.clip(a_cruise - a_e2e, 0., cap)) if allowed else 0.
      self.boost = float(np.clip(target, self.boost - RAMP_DOWN * DT_MDL, self.boost + RAMP_UP * DT_MDL))
    return a_e2e + self.boost
