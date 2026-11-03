"""
FM Pilot: Turn Boost and Turn Hold, for blinker turns at low speed.

Both act only while a turn desire is active (the blinker turn that modeld's DesireHelper publishes as
modelDataV2SP.laneTurnDirection, from AP's Lane Turn Desire or FM Pilot's Road Edge Turn Desire), only while openpilot is
steering, and only at low speed: full below 15 mph, fading out by 20 mph. They fade in and out over half a second, and they
only change the requested curvature, so every steering limit after them still applies.

  - Turn Boost scales the model's curvature request in the turn's direction by a gain per side (Turn Boost Left / Right,
    1.0 to 1.5). Newer models ask for wider turns than the truck needs and reach full torque late; a stronger request
    gets there sooner. It never reduces a request and never boosts a request that steers away from the turn.
  - Turn Hold keeps a model from letting go of a turn halfway (TSFDOM eases from about an 11 m radius to 30 m mid-turn).
    It tracks the turn's peak request and keeps the request at no less than 90% of it, a floor that drops by 15% of the
    peak per second after the peak. It releases for good once the truck has turned 75 degrees, after 8 s, when the driver
    steers, or when the turn ends.
"""
import math

import numpy as np

from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL

TurnDirection = custom.ModelDataV2SP.TurnDirection

PARAM_BOOST = "FM_TurnBoostEnabled"
PARAM_BOOST_LEFT = "FM_TurnBoostLeft"
PARAM_BOOST_RIGHT = "FM_TurnBoostRight"
PARAM_HOLD = "FM_TurnHoldEnabled"

BOOST_LIMITS = (1.0, 1.5)
FULL_SPEED = 15 * CV.MPH_TO_MS
MAX_SPEED = 20 * CV.MPH_TO_MS
FADE_TIME = 0.5  # s
HOLD_RATIO = 0.9
HOLD_DECAY = 0.15  # fraction of the peak per second, after the peak
HOLD_MAX_HEADING = math.radians(75.)
HOLD_MAX_TIME = 8.0  # s


def turn_sign(direction) -> int:
  """Curvature sign of a turn: left is positive."""
  if direction == TurnDirection.turnLeft:
    return 1
  if direction == TurnDirection.turnRight:
    return -1
  return 0


class FmTurnAssist:
  def __init__(self, params):
    self.params = params
    self.boost_enabled = False
    self.boost_left = 1.0
    self.boost_right = 1.0
    self.hold_enabled = False
    self.amount = 0.0  # 0..1 fade
    self._reset_turn(0)
    self.read_params()

  def read_params(self) -> None:
    def gain(key):
      try:
        value = float(self.params.get(key, return_default=True) or 1.0)
      except (TypeError, ValueError):
        value = 1.0
      return float(np.clip(value, *BOOST_LIMITS))
    self.boost_enabled = self.params.get_bool(PARAM_BOOST)
    self.boost_left = gain(PARAM_BOOST_LEFT)
    self.boost_right = gain(PARAM_BOOST_RIGHT)
    self.hold_enabled = self.params.get_bool(PARAM_HOLD)

  def _reset_turn(self, sign: int) -> None:
    self.sign = sign
    self.peak = 0.0
    self.since_peak = 0.0
    self.heading = 0.0
    self.turn_time = 0.0
    self.hold_released = False

  @property
  def enabled(self) -> bool:
    return self.boost_enabled or self.hold_enabled

  def update(self, desired_curvature: float, turn_direction, v_ego: float, actual_curvature: float,
             steering_pressed: bool, lat_active: bool) -> float:
    sign = turn_sign(turn_direction) if self.enabled else 0
    if sign != self.sign:
      self._reset_turn(sign)

    speed_scale = float(np.interp(v_ego, [FULL_SPEED, MAX_SPEED], [1.0, 0.0]))
    target = speed_scale if (sign != 0 and lat_active) else 0.0
    step = DT_CTRL / FADE_TIME
    self.amount = float(np.clip(target, self.amount - step, self.amount + step))
    if self.amount <= 0.0 or sign == 0:
      return desired_curvature

    out = desired_curvature
    toward = sign * out  # positive when the request turns the turn's way

    if self.boost_enabled and toward > 0.0:
      gain = self.boost_left if sign > 0 else self.boost_right
      out = out * (1.0 + (gain - 1.0) * self.amount)
      toward = sign * out

    if self.hold_enabled and not self.hold_released:
      self.turn_time += DT_CTRL
      self.heading += sign * actual_curvature * v_ego * DT_CTRL
      if steering_pressed or self.heading >= HOLD_MAX_HEADING or self.turn_time >= HOLD_MAX_TIME:
        self.hold_released = True
      else:
        if toward >= self.peak:
          self.peak, self.since_peak = toward, 0.0
        else:
          self.since_peak += DT_CTRL
        floor = self.peak * max(HOLD_RATIO - HOLD_DECAY * self.since_peak, 0.0)
        if toward < floor:
          out = sign * (toward + (floor - toward) * self.amount)

    return out
