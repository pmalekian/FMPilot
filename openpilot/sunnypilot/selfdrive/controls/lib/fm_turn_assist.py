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
  - Turn Lead starts the turn on time when a model turns in late. Between about 7 and 16 mph it reads the curve of the
    planned path 1.3 s ahead (4 to 14 m) and, if that bends harder toward the turn than the model's request, asks for
    85% of it instead. It fades out as the truck's actual curve catches up (gone once the truck turns as tightly as the
    path asks), and stays out when the truck is about to stop short of that point or the model steers away from the turn.
    Its request never goes tighter than Turn Path Tightest Radius (8 m at the tightest).
    Ported from StarPilot's Model Turn Assist "turn lead" (lateral/model_turn_assist.py, StarPilot Dom branch, Nov 2026,
    MIT), with two FM Pilot changes: a smooth speed fade in place of StarPilot's hard cutoff at 7 m/s, and the radius cap
    as a setting (StarPilot: fixed 0.12 1/m, about 8 m) defaulting gentler, to 12 m.

Turn Lead and Turn Hold can both be on, though one at a time makes a turn easier to judge. Turn Lead runs before Turn
Hold, so Turn Hold holds whatever Turn Lead asked for.
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
PARAM_LEAD = "FM_TurnLeadEnabled"
PARAM_MIN_RADIUS = "FM_TurnPathMinRadius"

BOOST_LIMITS = (1.0, 1.5)
FULL_SPEED = 15 * CV.MPH_TO_MS
MAX_SPEED = 20 * CV.MPH_TO_MS
FADE_TIME = 0.5  # s
HOLD_RATIO = 0.9
HOLD_DECAY = 0.15  # fraction of the peak per second, after the peak
HOLD_MAX_HEADING = math.radians(75.)
HOLD_MAX_TIME = 8.0  # s

# Turn Lead (from StarPilot's TURN_LEAD_*)
LEAD_TIME = 1.3  # s of travel to the path point it reads
LEAD_DIST = (4.0, 14.0)  # m
LEAD_PROBE_SPAN = 3.0  # m: a second point this much further, the gentler of the two is used
LEAD_SPEED_BP = [3.0, 4.0, 6.0, 7.0]  # m/s, about 6.7 / 9 / 13.4 / 15.7 mph
LEAD_SPEED_V = [0.0, 1.0, 1.0, 0.0]
LEAD_SCALE = 0.85
LEAD_ENGAGED_FRAC = 0.5  # full until the truck turns half as tightly as the path asks, gone when it turns as tightly
LEAD_MODEL_OPPOSE = 0.003  # 1/m: no Lead while the model steers away from the turn by more than this
LEAD_STOP_MARGIN = 1.5
LEAD_DECEL_GATE = -0.5  # m/s^2

MIN_RADIUS_LIMITS = (8.0, 20.0)  # m
MIN_RADIUS_DEFAULT = 12.0


def plan_circle_curvature(xs, ys, lookahead: float) -> float:
  """Curvature of the circle from the truck through the first planned point at least `lookahead` m away (2y/d^2,
  positive to the right like desiredCurvature). From StarPilot's _plan_circle_curvature."""
  px, py = 0.0, 0.0
  for x, y in zip(xs, ys, strict=False):
    try:
      x, y = float(x), float(y)
    except (TypeError, ValueError, OverflowError):
      return 0.0
    if not (math.isfinite(x) and math.isfinite(y)):
      return 0.0
    px, py = x, y
    if math.hypot(x, y) >= lookahead:
      break
  d2 = px * px + py * py
  if d2 < 1.0:
    return 0.0
  return 2.0 * py / d2


def plan_dual_probe(model_v2, d_near: float, d_far: float) -> float:
  """The gentler of the path's curvature at two distances, 0 if they disagree on direction. From StarPilot."""
  try:
    xs, ys = model_v2.position.x, model_v2.position.y
  except AttributeError:
    return 0.0
  near = plan_circle_curvature(xs, ys, d_near)
  far = plan_circle_curvature(xs, ys, d_far)
  if near * far <= 0.0:
    return 0.0
  return near if abs(near) < abs(far) else far


def turn_sign(direction) -> int:
  """Curvature sign of a turn. desiredCurvature is positive to the RIGHT (calibrated frame is forward-right-down;
  controlsd's curvature is -VM.calc_curvature of a left-positive steering angle), so a right turn is +1."""
  if direction == TurnDirection.turnRight:
    return 1
  if direction == TurnDirection.turnLeft:
    return -1
  return 0


class FmTurnAssist:
  def __init__(self, params):
    self.params = params
    self.boost_enabled = False
    self.boost_left = 1.0
    self.boost_right = 1.0
    self.hold_enabled = False
    self.lead_enabled = False
    self.max_curvature = 1.0 / MIN_RADIUS_DEFAULT
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
    self.lead_enabled = self.params.get_bool(PARAM_LEAD)
    try:
      radius = float(self.params.get(PARAM_MIN_RADIUS, return_default=True) or MIN_RADIUS_DEFAULT)
    except (TypeError, ValueError):
      radius = MIN_RADIUS_DEFAULT
    if not math.isfinite(radius):
      radius = MIN_RADIUS_DEFAULT
    self.max_curvature = 1.0 / float(np.clip(radius, *MIN_RADIUS_LIMITS))

  def _reset_turn(self, sign: int) -> None:
    self.sign = sign
    self.peak = 0.0
    self.since_peak = 0.0
    self.heading = 0.0
    self.turn_time = 0.0
    self.hold_released = False

  @property
  def enabled(self) -> bool:
    return self.boost_enabled or self.hold_enabled or self.lead_enabled

  def lead_curvature(self, model_v2, sign: int, v_ego: float, a_ego: float, actual_curvature: float) -> float:
    """Turn Lead's request toward the turn (>= 0, before the fade), or 0."""
    speed_w = float(np.interp(v_ego, LEAD_SPEED_BP, LEAD_SPEED_V))
    if model_v2 is None or speed_w <= 0.0:
      return 0.0
    d_near = float(np.clip(LEAD_TIME * v_ego, *LEAD_DIST))
    if a_ego < LEAD_DECEL_GATE and v_ego ** 2 / (2.0 * -a_ego) < LEAD_STOP_MARGIN * d_near:
      return 0.0  # stopping short of the point it reads
    lead = sign * plan_dual_probe(model_v2, d_near, d_near + LEAD_PROBE_SPAN) * LEAD_SCALE
    lead = min(lead, self.max_curvature)
    if lead <= 0.0:
      return 0.0
    engaged = max(sign * actual_curvature, 0.0) / lead
    engage_w = float(np.clip((1.0 - engaged) / (1.0 - LEAD_ENGAGED_FRAC), 0.0, 1.0))
    return lead * speed_w * engage_w

  def update(self, desired_curvature: float, turn_direction, v_ego: float, actual_curvature: float,
             steering_pressed: bool, lat_active: bool, model_v2=None, a_ego: float = 0.0,
             lane_changing: bool = False) -> float:
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
      gain = self.boost_right if sign > 0 else self.boost_left
      out = out * (1.0 + (gain - 1.0) * self.amount)
      toward = sign * out

    if self.lead_enabled and not lane_changing and toward > -LEAD_MODEL_OPPOSE:
      lead = self.lead_curvature(model_v2, sign, v_ego, a_ego, actual_curvature)
      if lead > max(toward, 0.0):
        out = sign * (toward + (lead - toward) * self.amount)
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
