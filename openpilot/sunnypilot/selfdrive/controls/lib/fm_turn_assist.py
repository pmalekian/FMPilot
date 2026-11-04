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
    peak per second after the peak. It releases for good once the truck has turned 75 degrees, after 8 s of moving, when
    the driver steers, or when the turn ends.
    Option Hold Path Source: when starting a turn from a stop or a crawl (under about 2 mph), also take the turn from the
    planned path, so a model whose steering request still says straight there turns anyway. It uses 85% of the gentler
    of the path's curves at 4 m and 7 m, weighted down unless the path has already turned more than 10 degrees within
    1.5 m (none past 5 m: the corner must be right there) and reaches at least 12 m ahead (none under 7 m), and never
    tighter than Turn Path Tightest Radius. Turn Hold then holds that as usual. From StarPilot's Model Turn Assist
    curvature hold (plan source), same thresholds. Time stopped (waiting to turn) doesn't count toward Turn Hold's 8 s.
    Option Hold Adds Your Steering: when you steer into the turn (hand torque toward the turn while the truck is turning
    that way), Turn Hold takes the truck's current curve as the turn to hold, up to full lock (Turn Path Tightest Radius
    doesn't apply: it is your curve), instead of releasing, so it keeps the help you gave once you let go and doesn't
    push back toward a wider turn while you hold the wheel in. Steering into the turn never releases it, even before the
    truck has started turning; steering against the turn still releases it at once. Hand
    torque toward the turn is the same direction openpilot steers, so the wheel's own reaction (which reads against
    openpilot) can't trigger it. From StarPilot's Model Turn Assist curvature hold (driver-confirmed wind).
  - Turn Lead starts the turn on time when a model turns in late. Between Turn Lead Min Speed (2 to 7 mph, default 7;
    full strength about 2.2 mph above it) and about 16 mph it reads the curve of the planned path 1.3 s ahead (4 to
    14 m) and, if that bends harder toward the turn than the model's request, asks for 85% of it instead. It fades out
    as the truck's actual curve catches up (gone once the truck turns as tightly as the path asks), and stays out when
    the truck is about to stop short of that point or the model steers away from the turn. Its request never goes
    tighter than Turn Path Tightest Radius (8 m at the tightest).
    Ported from StarPilot's Model Turn Assist "turn lead" (lateral/model_turn_assist.py, StarPilot Dom branch, Nov 2026,
    MIT), with FM Pilot changes: a smooth speed fade in place of StarPilot's hard cutoff at 7 m/s, the low end as a
    setting (StarPilot: fixed 3 m/s, about 6.7 mph) so it can reach turns pulling away from a stop, and the radius cap
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
PARAM_LEAD_MIN_SPEED = "FM_TurnLeadMinSpeed"  # mph
PARAM_HOLD_PATH = "FM_TurnHoldPathSource"
PARAM_HOLD_DRIVER = "FM_TurnHoldDriverAdds"
PARAM_MIN_RADIUS = "FM_TurnPathMinRadius"

BOOST_LIMITS = (1.0, 1.5)
FULL_SPEED = 15 * CV.MPH_TO_MS
MAX_SPEED = 20 * CV.MPH_TO_MS
FADE_TIME = 0.5  # s
HOLD_RATIO = 0.9
HOLD_DECAY = 0.15  # fraction of the peak per second, after the peak
HOLD_MAX_HEADING = math.radians(75.)
HOLD_MAX_TIME = 8.0  # s of moving
HOLD_MOVING_SPEED = 0.5  # m/s

# Turn Lead (from StarPilot's TURN_LEAD_*)
LEAD_TIME = 1.3  # s of travel to the path point it reads
LEAD_DIST = (4.0, 14.0)  # m
LEAD_PROBE_SPAN = 3.0  # m: a second point this much further, the gentler of the two is used
LEAD_MIN_SPEED_LIMITS = (2.0, 7.0)  # mph, Turn Lead Min Speed setting; 7 is about StarPilot's 3 m/s
LEAD_MIN_SPEED_DEFAULT = 7.0  # mph
LEAD_FADE_IN = 1.0  # m/s (about 2.2 mph) from the min speed to full strength
LEAD_FULL_TO, LEAD_MAX_SPEED = 6.0, 7.0  # m/s, about 13.4 and 15.7 mph
LEAD_SCALE = 0.85
LEAD_ENGAGED_FRAC = 0.5  # full until the truck turns half as tightly as the path asks, gone when it turns as tightly
LEAD_MODEL_OPPOSE = 0.003  # 1/m: no Lead while the model steers away from the turn by more than this
LEAD_STOP_MARGIN = 1.5
LEAD_DECEL_GATE = -0.5  # m/s^2

# Turn Hold path source (from StarPilot's CURVATURE_HOLD_PLAN_* / ONSET_* / REACH_*)
HOLD_PATH_MAX_SPEED = 2.0 * CV.MPH_TO_MS
HOLD_PATH_LOOKAHEAD = (4.0, 7.0)  # m
HOLD_PATH_SCALE = 0.85
HOLD_PATH_ONSET_HEADING = 10.0  # deg
HOLD_PATH_ONSET = (1.5, 5.0)  # m: full weight if the path starts turning within 1.5 m, none past 5 m
HOLD_PATH_REACH = (7.0, 12.0)  # m: none if the path reaches less than 7 m ahead, full from 12 m

HOLD_DRIVER_MIN_CURVATURE = 0.003  # 1/m: the truck must already be turning the turn's way (StarPilot's CONFIRM_MIN)
HOLD_DRIVER_MAX_CURVATURE = 0.25  # 1/m (4 m): beyond the R1S's full lock (about 6.6 m), only guards a bad reading

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


def plan_turn_onset(model_v2) -> float:
  """Distance to the first planned point where the path heads more than 10 degrees off straight. From StarPilot."""
  xs, ys = model_v2.position.x, model_v2.position.y
  for i in range(2, min(len(xs), len(ys))):
    dx, dy = xs[i] - xs[i - 1], ys[i] - ys[i - 1]
    if abs(dx) < 0.001 and abs(dy) < 0.001:
      continue
    if abs(math.degrees(math.atan2(dy, dx))) > HOLD_PATH_ONSET_HEADING:
      return math.hypot(xs[i], ys[i])
  return math.inf


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
    self.lead_min_speed = LEAD_MIN_SPEED_DEFAULT * CV.MPH_TO_MS
    self.hold_path = False
    self.hold_driver = False
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
      mph = float(self.params.get(PARAM_LEAD_MIN_SPEED, return_default=True) or LEAD_MIN_SPEED_DEFAULT)
    except (TypeError, ValueError):
      mph = LEAD_MIN_SPEED_DEFAULT
    if not math.isfinite(mph):
      mph = LEAD_MIN_SPEED_DEFAULT
    self.lead_min_speed = float(np.clip(mph, *LEAD_MIN_SPEED_LIMITS)) * CV.MPH_TO_MS
    self.hold_path = self.params.get_bool(PARAM_HOLD_PATH)
    self.hold_driver = self.params.get_bool(PARAM_HOLD_DRIVER)
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
    lo = self.lead_min_speed
    speed_w = float(np.interp(v_ego, [lo, lo + LEAD_FADE_IN, LEAD_FULL_TO, LEAD_MAX_SPEED], [0.0, 1.0, 1.0, 0.0]))
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

  def hold_path_curvature(self, model_v2, sign: int) -> float:
    """Turn Hold's path source toward the turn (>= 0, before the fade), or 0."""
    try:
      xs = model_v2.position.x
      reach = float(xs[-1]) if len(xs) else 0.0
      path = sign * plan_dual_probe(model_v2, *HOLD_PATH_LOOKAHEAD) * HOLD_PATH_SCALE
      onset = plan_turn_onset(model_v2)
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
      return 0.0
    if not (math.isfinite(reach) and math.isfinite(path)) or path <= 0.0:
      return 0.0
    onset_w = float(np.interp(onset, HOLD_PATH_ONSET, [1.0, 0.0])) if math.isfinite(onset) else 0.0
    reach_w = float(np.interp(reach, HOLD_PATH_REACH, [0.0, 1.0]))
    return min(path, self.max_curvature) * onset_w * reach_w

  def update(self, desired_curvature: float, turn_direction, v_ego: float, actual_curvature: float,
             steering_pressed: bool, lat_active: bool, model_v2=None, a_ego: float = 0.0,
             lane_changing: bool = False, steering_torque: float = 0.0) -> float:
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
      if v_ego >= HOLD_MOVING_SPEED:
        self.turn_time += DT_CTRL  # time stopped (waiting to turn) doesn't count toward the 8 s
      self.heading += sign * actual_curvature * v_ego * DT_CTRL
      # Hand torque is positive to the LEFT and curvature positive to the RIGHT, so torque into the turn is sign * torque < 0
      # Steering into the turn never releases the hold (you often start turning the wheel before the truck turns);
      # the truck's curve is only adopted once it is actually turning that way.
      into_turn = steering_pressed and self.hold_driver and sign * steering_torque < 0.0
      driver_adds = into_turn and sign * actual_curvature > HOLD_DRIVER_MIN_CURVATURE
      if (steering_pressed and not into_turn) or self.heading >= HOLD_MAX_HEADING or self.turn_time >= HOLD_MAX_TIME:
        self.hold_released = True
      else:
        if driver_adds:
          wound = min(sign * actual_curvature, HOLD_DRIVER_MAX_CURVATURE)  # your curve, not the slider's
          if wound > toward:
            out = sign * (toward + (wound - toward) * self.amount)
            toward = sign * out
        if self.hold_path and model_v2 is not None and v_ego < HOLD_PATH_MAX_SPEED:
          path = self.hold_path_curvature(model_v2, sign)
          if path > toward:
            out = sign * (toward + (path - toward) * self.amount)
            toward = sign * out
        if toward >= self.peak:
          self.peak, self.since_peak = toward, 0.0
        else:
          self.since_peak += DT_CTRL
        floor = self.peak * max(HOLD_RATIO - HOLD_DECAY * self.since_peak, 0.0)
        if toward < floor:
          out = sign * (toward + (floor - toward) * self.amount)

    return out
