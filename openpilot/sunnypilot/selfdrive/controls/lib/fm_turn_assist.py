"""
FM Pilot: Turn Boost and Turn Hold, for blinker turns at low speed.

Both act only while a turn desire is active (the blinker turn that modeld's DesireHelper publishes as
modelDataV2SP.laneTurnDirection, from AP's Lane Turn Desire or FM Pilot's Road Edge Turn Desire), only while openpilot is
steering, and only at low speed: full below 15 mph, fading out by 20 mph. They fade with speed over half a second. An active hold also fades when the turn desire ends. They
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
    Option Hold Through Early Correction: at the start of a turn (under 5 mph and before the truck has turned 20
    degrees), steering against the turn, for example straightening the truck before turning, doesn't release the hold.
    While you steer, the model's request passes through and the hold's fade is paused; once you let go it carries on
    from the same peak. Past 5 mph or 20 degrees, steering against the turn releases it as before. Straightening also
    doesn't count against the 75 degrees.
    Option Ignore Brief Hand Blips (on by default): steering against the turn releases Turn Hold only once it has lasted
    0.25 s, or at once with 0.5 Nm or more against the turn. A single frame read as pressed while your hand relaxes
    (00000070--77 +33.78 s: +0.14 Nm at 12.7 mph) released the hold for the rest of that turn.
  - Turn Lead starts the turn on time when a model turns in late. Between Turn Lead Min Speed (2 to 7 mph, default 7;
    full strength about 2.2 mph above it) and about 16 mph it reads the curve of the planned path 1.3 s ahead (4 to
    14 m) and, if that bends harder toward the turn than the model's request, asks for 85% of it instead. It fades out
    as the truck's actual curve catches up (gone once the truck turns as tightly as the path asks), and stays out when
    the truck is about to stop short of that point or the model steers away from the turn. Its request never goes
    tighter than Turn Path Tightest Radius (8 m at the tightest).
    Option Adaptive Look-Ahead (on by default): instead of a fixed 1.3 s, it reads the path as far ahead as the steering
    actually lags: 0.5 s for a gentle curve (50 m radius or wider) up to 0.9 s for a tight one (12.5 m or tighter),
    never closer than 3 m. That is about openpilot's 0.25 s steering delay plus the time the torque needs to build at
    the current ramp (about 4 counts per 10 ms). A fixed 1.3 s at 12 to 16 mph asks for the turn 7 to 9 m before it
    starts (it cut the curb on 0000003a--83).
    Option Start Distance gate (on by default, 5 m; 3 to 10 m): Turn Lead only acts once the point where the planned path
    has turned more than 10 degrees toward the turn is within that distance (fading in over the 2 m before it). That
    point is 2 to 3 m into a 12 m corner, so at 5 m the lead starts about 2 to 3 m before the corner itself. Once the truck
    is turning, the path ahead bends right away, so the gate stays open; once the turn straightens out it closes.
    Ported from StarPilot's Model Turn Assist "turn lead" (lateral/model_turn_assist.py, StarPilot Dom branch, Nov 2026,
    MIT), with FM Pilot changes: a smooth speed fade in place of StarPilot's hard cutoff at 7 m/s, the low end as a
    setting (StarPilot: fixed 3 m/s, about 6.7 mph) so it can reach turns pulling away from a stop, and the radius cap
    as a setting (StarPilot: fixed 0.12 1/m, about 8 m) defaulting gentler, to 12 m.

Turn Lead and Turn Hold can both be on, though one at a time makes a turn easier to judge. Turn Lead runs before Turn
Hold, so Turn Hold holds whatever Turn Lead asked for.

  - Smooth Turn Start (on by default): whatever Turn Boost, Lead, Hold and Turn Nudge add on top of the model's request
    eases in over about 0.3 s (a 0.12 s low-pass) instead of arriving in one step; it still drops at once, so handing back
    to the model is never delayed. A step makes openpilot's torque controller saturate and then swing the other way as
    the truck catches up (00000068--1 +24.3 s: Turn Lead stepped from 0.016 to 0.050 1/m in 0.1 s, the request went to
    the cap, then briefly steered out of the left turn at +25.4 s). The model's own request is never filtered.
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
PARAM_TURN_NUDGE = "FM_TurnNudgeEnabled"
PARAM_HOLD_DRIVER = "FM_TurnHoldDriverAdds"
PARAM_HOLD_EARLY = "FM_TurnHoldEarlyCorrection"
PARAM_MIN_RADIUS = "FM_TurnPathMinRadius"
PARAM_PHASE_IN = "FM_TurnPhaseIn"
PARAM_LEAD_ADAPTIVE = "FM_TurnLeadAdaptive"
PARAM_LEAD_GATE = "FM_TurnLeadGate"
PARAM_HOLD_DEBOUNCE = "FM_TurnHoldReleaseDebounce"
PARAM_LEAD_GATE_DIST = "FM_TurnLeadGateDist"  # m

BOOST_LIMITS = (1.0, 1.5)
FULL_SPEED = 15 * CV.MPH_TO_MS
MAX_SPEED = 20 * CV.MPH_TO_MS
FADE_TIME = 0.5  # s
HOLD_RATIO = 0.9
HOLD_DECAY = 0.15  # fraction of the peak per second, after the peak
HOLD_MAX_HEADING = math.radians(75.)
HOLD_MAX_TIME = 8.0  # s of moving
HOLD_MOVING_SPEED = 0.5  # m/s
HOLD_EARLY_MAX_SPEED = 5 * CV.MPH_TO_MS  # Hold Through Early Correction: the start of a turn is under this speed
HOLD_EARLY_MAX_HEADING = math.radians(20.)  # and before the truck has turned this much
# Ignore Brief Hand Blips: steering against the turn releases Turn Hold only once it lasts this long, or at once past
# this much hand torque against the turn (00000070--77 +33.78 s: one 10 ms frame read pressed at +0.14 Nm, 12.7 mph)
HOLD_AGAINST_TIME = 0.25  # s
HOLD_AGAINST_TORQUE = 0.5  # Nm


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
# Adaptive Look-Ahead: seconds of travel to read ahead, by how tight the path bends there (1/m)
LEAD_ADAPTIVE_CURVATURE = (0.02, 0.08)  # 50 m and 12.5 m radius
LEAD_ADAPTIVE_TIME = (0.5, 0.9)  # s
LEAD_ADAPTIVE_DIST = (3.0, 14.0)  # m
# Start Distance gate
LEAD_GATE_HEADING = 10.0  # deg toward the turn: where the path counts as turning
LEAD_GATE_DIST_LIMITS = (3.0, 10.0)  # m, setting
LEAD_GATE_DIST_DEFAULT = 5.0
LEAD_GATE_FADE = 2.0  # m: from no lead to full lead

# Turn Hold path source (from StarPilot's CURVATURE_HOLD_PLAN_* / ONSET_* / REACH_*)
HOLD_PATH_MAX_SPEED = 2.0 * CV.MPH_TO_MS
HOLD_PATH_LOOKAHEAD = (4.0, 7.0)  # m
HOLD_PATH_SCALE = 0.85
HOLD_PATH_ONSET_HEADING = 10.0  # deg
HOLD_PATH_ONSET = (1.5, 5.0)  # m: full weight if the path starts turning within 1.5 m, none past 5 m
HOLD_PATH_REACH = (7.0, 12.0)  # m: none if the path reaches less than 7 m ahead, full from 12 m

NUDGE_MIN_SPEED = 2.0 * CV.MPH_TO_MS
NUDGE_MAX_SPEED = 8.0 * CV.MPH_TO_MS
NUDGE_MIN_TORQUE = 0.8  # Nm into the signaled turn
NUDGE_CONFIRM_TIME = 0.15
NUDGE_DURATION = 0.6
NUDGE_FADE_TIME = 0.15
NUDGE_MAX_CURVATURE = 0.01  # 1/m added to the normal request

HOLD_DRIVER_MIN_CURVATURE = 0.003  # 1/m: the truck must already be turning the turn's way (StarPilot's CONFIRM_MIN)
HOLD_DRIVER_MAX_CURVATURE = 0.25  # 1/m (4 m): beyond the R1S's full lock (about 6.6 m), only guards a bad reading

# Smooth Turn Start: first-order low-pass on the extra request while it rises (about 90% after 0.3 s)
PHASE_IN_TAU = 0.12  # s
PHASE_IN_ALPHA = DT_CTRL / (PHASE_IN_TAU + DT_CTRL)

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


def plan_turn_onset_toward(model_v2, sign: int, heading_deg: float = LEAD_GATE_HEADING) -> float:
  """Distance to the first planned point where the path heads more than heading_deg toward the turn (right for sign +1:
  the calibrated frame's y is to the right), or inf."""
  try:
    xs, ys = model_v2.position.x, model_v2.position.y
    for i in range(1, min(len(xs), len(ys))):
      dx, dy = float(xs[i]) - float(xs[i - 1]), float(ys[i]) - float(ys[i - 1])
      if abs(dx) < 0.001 and abs(dy) < 0.001:
        continue
      if sign * math.degrees(math.atan2(dy, dx)) > heading_deg:
        return math.hypot(float(xs[i]), float(ys[i]))
  except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
    pass
  return math.inf


def plan_max_curvature(model_v2, sign: int, max_dist: float, spacing: float = 1.5) -> float:
  """Tightest bend toward the turn (>= 0, 1/m) along the planned path within max_dist: heading change between points
  at least `spacing` m apart, over the distance between their segment midpoints."""
  try:
    xs, ys = model_v2.position.x, model_v2.position.y
  except AttributeError:
    return 0.0
  pts = [(0.0, 0.0)]
  for x, y in zip(xs, ys, strict=False):
    try:
      x, y = float(x), float(y)
    except (TypeError, ValueError, OverflowError):
      return 0.0
    if not (math.isfinite(x) and math.isfinite(y)):
      return 0.0
    if math.hypot(x - pts[-1][0], y - pts[-1][1]) >= spacing:
      pts.append((x, y))
    if math.hypot(x, y) > max_dist:
      break
  best, prev, s = 0.0, None, 0.0
  for (x0, y0), (x1, y1) in zip(pts, pts[1:], strict=False):
    seg = math.hypot(x1 - x0, y1 - y0)
    heading, mid = math.atan2(y1 - y0, x1 - x0), s + seg / 2.0
    if prev is not None:
      best = max(best, sign * (heading - prev[0]) / (mid - prev[1]))
    prev, s = (heading, mid), s + seg
  return best


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
    self.hold_early = False
    self.max_curvature = 1.0 / MIN_RADIUS_DEFAULT
    self.phase_in = False
    self.lead_adaptive = False
    self.lead_gate = False
    self.lead_gate_dist = LEAD_GATE_DIST_DEFAULT
    self.hold_debounce = False
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
    self.hold_early = self.params.get_bool(PARAM_HOLD_EARLY)
    self.nudge_enabled = self.params.get_bool(PARAM_TURN_NUDGE)
    self.phase_in = self.params.get_bool(PARAM_PHASE_IN)
    self.lead_adaptive = self.params.get_bool(PARAM_LEAD_ADAPTIVE)
    self.lead_gate = self.params.get_bool(PARAM_LEAD_GATE)
    self.hold_debounce = self.params.get_bool(PARAM_HOLD_DEBOUNCE)
    try:
      gate = float(self.params.get(PARAM_LEAD_GATE_DIST, return_default=True) or LEAD_GATE_DIST_DEFAULT)
    except (TypeError, ValueError):
      gate = LEAD_GATE_DIST_DEFAULT
    if not math.isfinite(gate):
      gate = LEAD_GATE_DIST_DEFAULT
    self.lead_gate_dist = float(np.clip(gate, *LEAD_GATE_DIST_LIMITS))
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
    self.hold_exit_curvature = 0.0
    self.hold_exit_amount = 0.0
    self.turn_ended = False
    self.nudge_time = 0.0
    self.nudge_remaining = 0.0
    self.against_time = 0.0
    self.extra = 0.0  # Smooth Turn Start: what is applied on top of the model's request, toward the turn

  @property
  def enabled(self) -> bool:
    return self.boost_enabled or self.hold_enabled or self.lead_enabled

  def lead_curvature(self, model_v2, sign: int, v_ego: float, a_ego: float, actual_curvature: float) -> float:
    """Turn Lead's request toward the turn (>= 0, before the fade), or 0."""
    lo = self.lead_min_speed
    speed_w = float(np.interp(v_ego, [lo, lo + LEAD_FADE_IN, LEAD_FULL_TO, LEAD_MAX_SPEED], [0.0, 1.0, 1.0, 0.0]))
    if model_v2 is None or speed_w <= 0.0:
      return 0.0
    d_near = self.lead_distance(model_v2, sign, v_ego)
    if a_ego < LEAD_DECEL_GATE and v_ego ** 2 / (2.0 * -a_ego) < LEAD_STOP_MARGIN * d_near:
      return 0.0  # stopping short of the point it reads
    lead = sign * plan_dual_probe(model_v2, d_near, d_near + LEAD_PROBE_SPAN) * LEAD_SCALE
    lead = min(lead, self.max_curvature)
    if lead <= 0.0:
      return 0.0
    engaged = max(sign * actual_curvature, 0.0) / lead
    engage_w = float(np.clip((1.0 - engaged) / (1.0 - LEAD_ENGAGED_FRAC), 0.0, 1.0))
    gate_w = 1.0
    if self.lead_gate:
      onset = plan_turn_onset_toward(model_v2, sign)
      gate_w = float(np.interp(onset, [self.lead_gate_dist, self.lead_gate_dist + LEAD_GATE_FADE], [1.0, 0.0])) \
        if math.isfinite(onset) else 0.0
    return lead * speed_w * engage_w * gate_w

  def lead_distance(self, model_v2, sign: int, v_ego: float) -> float:
    """How far ahead Turn Lead reads the path: a fixed 1.3 s of travel, or with Adaptive Look-Ahead 0.5 to 0.9 s by how
    tightly the planned path bends toward the turn anywhere in the next 17 m."""
    if not self.lead_adaptive:
      return float(np.clip(LEAD_TIME * v_ego, *LEAD_DIST))
    bend = plan_max_curvature(model_v2, sign, LEAD_DIST[1] + LEAD_PROBE_SPAN)
    lead_time = float(np.interp(bend, LEAD_ADAPTIVE_CURVATURE, LEAD_ADAPTIVE_TIME))
    return float(np.clip(lead_time * v_ego, *LEAD_ADAPTIVE_DIST))

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

  def turn_nudge(self, out, sign, v_ego, a_ego, model_v2, steering_pressed, steering_torque, lat_active, lane_changing):
    stopping = a_ego < LEAD_DECEL_GATE and v_ego ** 2 / (-2.0 * a_ego) < HOLD_PATH_LOOKAHEAD[0]
    hand_torque = sign * steering_torque
    if (not self.nudge_enabled or not self.hold_enabled or not lat_active or lane_changing or stopping
        or not NUDGE_MIN_SPEED <= v_ego < NUDGE_MAX_SPEED or (steering_pressed and hand_torque > 0.0)):
      self.nudge_time = self.nudge_remaining = 0.0
      return out

    room = self.hold_path_curvature(model_v2, sign) - max(0.0, sign * out)
    if room <= LEAD_MODEL_OPPOSE:
      self.nudge_time = self.nudge_remaining = 0.0
      return out

    if steering_pressed and hand_torque <= -NUDGE_MIN_TORQUE:
      if self.nudge_time < NUDGE_CONFIRM_TIME:
        self.nudge_time += DT_CTRL
        if self.nudge_time >= NUDGE_CONFIRM_TIME:
          self.nudge_remaining = NUDGE_DURATION
    else:
      self.nudge_time = 0.0

    speed_weight = (NUDGE_MAX_SPEED - v_ego) / (NUDGE_MAX_SPEED - NUDGE_MIN_SPEED)
    fade = min(self.nudge_remaining / NUDGE_FADE_TIME, 1.0)
    self.nudge_remaining = max(0.0, self.nudge_remaining - DT_CTRL)
    return out + sign * min(room, NUDGE_MAX_CURVATURE) * speed_weight * fade

  def update(self, desired_curvature: float, turn_direction, v_ego: float, actual_curvature: float,
             steering_pressed: bool, lat_active: bool, model_v2=None, a_ego: float = 0.0,
             lane_changing: bool = False, steering_torque: float = 0.0) -> float:
    sign = turn_sign(turn_direction) if self.enabled else 0
    step = DT_CTRL / FADE_TIME
    if sign == 0 and self.hold_exit_amount > 0.0:
      self.turn_ended = True
      # Only carry the last applied hold into the exit. Do not update its peak or run Lead/Boost.
      self.amount = max(self.amount - step, 0.0)
      self.hold_exit_amount = max(self.hold_exit_amount - step, 0.0)
      driver_opposes = steering_pressed and not (self.hold_driver and self.sign * steering_torque < 0.0)
      if (self.hold_enabled and lat_active and not driver_opposes and not lane_changing
          and v_ego < MAX_SPEED and self.hold_exit_amount > 0.0):
        toward = self.sign * desired_curvature
        correction = max(self.hold_exit_curvature - toward, 0.0)
        return desired_curvature + self.sign * correction * self.hold_exit_amount
      self._reset_turn(0)
      return desired_curvature

    if sign != self.sign or self.turn_ended:
      self._reset_turn(sign)

    speed_scale = float(np.interp(v_ego, [FULL_SPEED, MAX_SPEED], [1.0, 0.0]))
    target = speed_scale if (sign != 0 and lat_active) else 0.0
    self.amount = float(np.clip(target, self.amount - step, self.amount + step))
    self.hold_exit_amount = 0.0
    if self.amount <= 0.0 or sign == 0:
      self.extra = 0.0
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

    before_hold = out
    if self.hold_enabled and not self.hold_released:
      if v_ego >= HOLD_MOVING_SPEED:
        self.turn_time += DT_CTRL  # time stopped (waiting to turn) doesn't count toward the 8 s
      self.heading += sign * actual_curvature * v_ego * DT_CTRL
      # Hand torque is positive to the LEFT and curvature positive to the RIGHT, so torque into the turn is sign * torque < 0
      # Steering into the turn never releases the hold (you often start turning the wheel before the truck turns);
      # the truck's curve is only adopted once it is actually turning that way.
      into_turn = steering_pressed and self.hold_driver and sign * steering_torque < 0.0
      driver_adds = into_turn and sign * actual_curvature > HOLD_DRIVER_MIN_CURVATURE
      against = steering_pressed and not into_turn
      self.against_time = self.against_time + DT_CTRL if against else 0.0
      if against and self.hold_debounce:
        # Ignore Brief Hand Blips: a press against the turn only counts once it lasts or is firm
        against = self.against_time >= HOLD_AGAINST_TIME - 1e-6 or sign * steering_torque >= HOLD_AGAINST_TORQUE
      early_correction = (steering_pressed and sign * steering_torque > 0.0 and self.hold_early and v_ego < HOLD_EARLY_MAX_SPEED
                          and self.heading < HOLD_EARLY_MAX_HEADING and self.turn_time < HOLD_MAX_TIME)
      if early_correction:
        # Straightening before the turn: pass the request through, pause the fade, keep the peak; don't release
        self.heading = max(self.heading, 0.0)
      elif against or self.heading >= HOLD_MAX_HEADING or self.turn_time >= HOLD_MAX_TIME:
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

    if lat_active and sign * (out - before_hold) > 0.0:
      self.hold_exit_curvature = sign * out
      self.hold_exit_amount = 1.0
    # The timing nudge never feeds the hold peak or its signal-cancel fade.
    out = self.turn_nudge(out, sign, v_ego, a_ego, model_v2, steering_pressed, steering_torque, lat_active, lane_changing)
    return self._smooth_start(desired_curvature, out, sign)

  def _smooth_start(self, desired_curvature: float, out: float, sign: int) -> float:
    """Smooth Turn Start: the extra request toward the turn rises through a low-pass and drops at once."""
    target = sign * (out - desired_curvature)
    if self.phase_in and target > self.extra:
      self.extra += (target - self.extra) * PHASE_IN_ALPHA
    else:
      self.extra = target
    if self.hold_exit_amount > 0.0:
      # a blinker cancel fades out from what was actually applied, never from more
      self.hold_exit_curvature = min(self.hold_exit_curvature, sign * desired_curvature + self.extra)
    return desired_curvature + sign * self.extra
