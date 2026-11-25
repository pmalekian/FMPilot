"""
FM Pilot: Turn Hold and Smooth Turn Start, for blinker turns at low speed, and the Turn Assist log (fmControls.turnAssist).

Turn Hold acts only while a turn desire is active (the blinker turn that modeld's DesireHelper publishes as
modelDataV2SP.laneTurnDirection, from sunnypilot's Lane Turn Desire), only while openpilot is steering, and only at low
speed: full below 15 mph, fading out by 20 mph. It fades with speed over half a second. An active hold also fades over
half a second when the turn desire ends. It only changes the requested curvature, so every steering limit after it still
applies.

Turn Hold keeps a model from letting go of a turn halfway (TSFDOM eases from about an 11 m radius to 30 m mid-turn). It
tracks the turn's peak request and keeps the request at no less than 90% of it, a floor that drops by 15% of the peak per
second after the peak. It releases for good once the truck has turned 75 degrees, after 8 s of moving, when the driver
steers against the turn, or when the turn ends. Built into it (fm-dev-sync folded these toggles, FM_FOLDED_DEFAULTS.md;
their old keys are not read):
  - Turn Nudge: a deliberate nudge into the turn (0.8 Nm for 0.15 s) at 2 to 8 mph, when the planned path right ahead
    already bends that way, adds up to 0.01 1/m for 0.6 s (less toward 8 mph). Steering against the turn, pressing
    into it, stopping or a lane change cancel it. It never feeds the hold's peak or its signal-cancel fade.
  - Hold Through Early Correction: at the start of a turn (under 5 mph and before the truck has turned 20 degrees),
    steering against the turn, for example straightening the truck before turning, doesn't release the hold. While you
    steer, the model's request passes through and the hold's fade is paused; once you let go it carries on from the
    same peak. Straightening doesn't count against the 75 degrees.
  - Ignore Brief Hand Blips: steering against the turn releases Turn Hold only once it has lasted 0.25 s, or at once
    with 1.5 Nm or more against the turn. A single frame read as pressed while your hand relaxes (00000070--77 +33.78 s:
    +0.14 Nm at 12.7 mph) used to release the hold for the rest of that turn.
  - Hold Starts at Turn-In: Turn Hold isn't used up before the turn. Its 8 s only count once the truck has turned 5
    degrees; until it has turned 10 degrees, steering against the turn (a hand resting on the wheel, or the wheel's own
    reaction as the blinker comes on) passes through like Hold Through Early Correction, at any speed Turn Hold works
    at, instead of releasing it. Before 10 degrees only a press against the turn lasting 1.5 s releases it (you aren't
    turning after all; 00000073's pre-turn presses lasted up to 0.97 s).
  - Hold Only Counts While Steering: while openpilot isn't steering (the truck stopped: no steering under 0.3 m/s; a
    steering fault; lateral paused) Turn Hold keeps no books: its peak, its 8 s and its 75 degrees stay as they were and
    it adds nothing. controlsd passes the truck's own curvature as the request then, which must not become the turn's
    peak.
  - Hold Ignores Crawl Spikes: below 5 mph the hold's peak only learns a request kept up for 0.3 s, so a split-second
    jump in the model's request while pulling away isn't held as the turn.
Options (Turn Hold Settings, off by default):
  - Hold Path Source: when starting a turn from a stop or a crawl (under about 2 mph), also take the turn from the planned
    path, so a model whose steering request still says straight there turns anyway. It uses 85% of the gentler of the
    path's curves at 4 m and 7 m, weighted down unless the path has already turned more than 10 degrees within 1.5 m
    (none past 5 m: the corner must be right there) and reaches at least 12 m ahead (none under 7 m), and never tighter
    than Turn Path Tightest Radius (FM_TurnPathMinRadius, 8 to 20 m, default 12; Turn Nudge's path uses it too). Turn
    Hold then holds that as usual. From StarPilot's Model Turn Assist curvature hold (plan source), same thresholds.
  - Hold Adds Your Steering: when you steer into the turn (hand torque toward the turn while the truck is turning that
    way), Turn Hold takes the truck's current curve as the turn to hold, up to full lock (Turn Path Tightest Radius
    doesn't apply: it is your curve), instead of releasing, so it keeps the help you gave once you let go and doesn't
    push back toward a wider turn while you hold the wheel in. Steering into the turn never releases it, even before the
    truck has started turning; steering against the turn still releases it as above. Hand torque toward the turn is the
    same direction openpilot steers, so the wheel's own reaction (which reads against openpilot) can't trigger it. From
    StarPilot's Model Turn Assist curvature hold (driver-confirmed wind).
  - Hold Follows the Model's Exit: once the truck has turned 45 degrees, when the model asks for less than half of the
    hold's peak (it is letting go of the turn) and you aren't pressing into the turn, Turn Hold fades out over half a
    second instead of keeping its floor until 75 degrees. In 00000073 (--8 +34.8 s, --8 +46.5 s, --2 +45.8 s) the floor
    kept the request 1 to 2.5 s tighter than the model until he steered out.
  - Stop Before Turning Starts Over: a stop before the truck has turned 10 degrees (a queue, a creep to the line) starts
    the turn over, so nothing learned on the way (the hold's peak, a release) carries into the turn.

Smooth Turn Start (FM_TurnPhaseIn, off by default): whatever Turn Hold (and its nudge) adds on top of the model's request
eases in over about 0.3 s (a 0.12 s low-pass) instead of arriving in one step; it still drops at once, so handing back to
the model is never delayed. A step makes openpilot's torque controller saturate and then swing the other way as the truck
catches up (00000068--1 +24.3 s). The model's own request is never filtered.

Hold Path Source and Hold Adds Your Steering come from StarPilot's Model Turn Assist curvature hold
(openpilot/starpilot/lateral/model_turn_assist.py, Dom branch, Nov 2026, firestar5683 and StarPilot contributors, MIT).
Ported from fm-dev-nav-settle ca08aac9 (fm_turn_assist.py) with the toggles above folded; Turn Boost is dropped. Turn Lead
(plan PR 11) runs before Turn Hold in the same update, so Turn Hold holds whatever it asks for, and Finish Turn After
Signal Cancels (plan PR 12) supplies the turn's sign after the blinker cancels: both plug in where update() says so.
"""
import math
from collections import deque

import numpy as np

from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.fm_math import step_toward

TurnDirection = custom.ModelDataV2SP.TurnDirection

PARAM_HOLD = "FM_TurnHoldEnabled"
PARAM_HOLD_PATH = "FM_TurnHoldPathSource"
PARAM_HOLD_DRIVER = "FM_TurnHoldDriverAdds"
PARAM_HOLD_FOLLOW_EXIT = "FM_TurnHoldFollowExit"
PARAM_RESTART_AFTER_STOP = "FM_TurnRestartAfterStop"
PARAM_PHASE_IN = "FM_TurnPhaseIn"  # Smooth Turn Start
PARAM_MIN_RADIUS = "FM_TurnPathMinRadius"  # Turn Path Tightest Radius (shared with Turn Lead)
# Folded into Turn Hold on fm-dev-sync (always on with it). Their old keys stay declared and are never read.
FOLDED_KEYS = ("FM_TurnNudgeEnabled", "FM_TurnHoldEarlyCorrection", "FM_TurnHoldReleaseDebounce",
               "FM_TurnHoldStartsAtTurnIn", "FM_TurnHoldSteeringOnly", "FM_TurnHoldSteadyPeak")

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
# HOLD_AGAINST_TORQUE_FIRM against the turn (00000070--77 +33.78 s: one 10 ms frame read pressed at +0.14 Nm, 12.7 mph)
HOLD_AGAINST_TIME = 0.25  # s
# Hold Follows the Model's Exit: past this much turned, once the model asks less than this share of the hold's peak (and
# you aren't pressing into the turn), Turn Hold fades out over FADE_TIME. 00000073: --8 +34.8 s, --8 +46.5 s and
# --2 +45.8 s the model let go and the floor kept the request 1-2.5 s tighter until he steered out.
HOLD_EXIT_TURNED = math.radians(45.)
HOLD_EXIT_FRAC = 0.5
# Hold Starts at Turn-In: Turn Hold isn't spent before the turn. 00000073: --3 +13.4 s the 8 s ran out on the approach
# (blinker on at 20 mph), --8 +13.44 s a 0.62 Nm press as the blinker came on, --5 +8.64 s one 10 ms frame at 1.01 Nm
# (most likely the wheel's reaction to openpilot's torque) each released it before the truck had turned.
HOLD_CLOCK_TURNED = math.radians(5.)   # the 8 s only counts once the truck has turned this much
HOLD_START_TURNED = math.radians(10.)  # before this, a press against the turn passes through without releasing
HOLD_AGAINST_TORQUE_FIRM = 1.5  # Nm: a press against the turn this firm releases at once (else after 0.25 s)
HOLD_START_RELEASE_TIME = 1.5  # s: before HOLD_START_TURNED, a press against the turn this long still releases (you
                               # aren't turning after all); 00000073's pre-turn presses lasted up to 0.97 s
# Hold Ignores Crawl Spikes: below this speed Turn Hold's peak only learns what has been asked for HOLD_STEADY_TIME
HOLD_STEADY_MAX_SPEED = 5 * CV.MPH_TO_MS
HOLD_STEADY_TIME = 0.3  # s
HOLD_STEADY_FRAMES = int(round(HOLD_STEADY_TIME / DT_CTRL))

LEAD_DECEL_GATE = -0.5  # m/s^2: braking harder than this counts as stopping (Turn Nudge; Turn Lead uses it too)
LEAD_MODEL_OPPOSE = 0.003  # 1/m: Turn Nudge needs more room than this between the path and the request

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

# Logged in fmControls.turnAssist: where the turn came from and why Turn Hold let go (same numbers as the log enums).
# SOURCE_CARRY is Finish Turn After Signal Cancels (plan PR 12).
SOURCE_NONE, SOURCE_DESIRE, SOURCE_CARRY = 0, 1, 2
RELEASE_NONE, RELEASE_DRIVER, RELEASE_TURNED, RELEASE_TIME, RELEASE_MODEL_EXIT = 0, 1, 2, 3, 4


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
    self.hold_enabled = False
    self.hold_path = False
    self.hold_driver = False
    self.hold_follow_exit = False
    self.restart_after_stop = False
    self.phase_in = False
    self.max_curvature = 1.0 / MIN_RADIUS_DEFAULT
    self.restarts = 0  # Stop Before Turning Starts Over: times this turn was started over (logged)
    self.amount = 0.0  # 0..1 fade
    self._reset_turn(0)
    self._clear_log(0.0)
    self.read_params()

  def read_params(self) -> None:
    self.hold_enabled = self.params.get_bool(PARAM_HOLD)
    self.hold_path = self.params.get_bool(PARAM_HOLD_PATH)
    self.hold_driver = self.params.get_bool(PARAM_HOLD_DRIVER)
    self.hold_follow_exit = self.params.get_bool(PARAM_HOLD_FOLLOW_EXIT)
    self.restart_after_stop = self.params.get_bool(PARAM_RESTART_AFTER_STOP)
    self.phase_in = self.params.get_bool(PARAM_PHASE_IN)
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
    self.turned = 0.0  # rad the truck has turned toward the turn since it started (every turn, hold or not)
    self.peak_actual = 0.0  # 1/m: the truck's tightest curve toward the turn so far
    self.release_reason = RELEASE_NONE
    self.exit_fade = 0.0  # Hold Follows the Model's Exit: 1 -> 0 while the released hold fades out
    self.exit_fade_curvature = 0.0  # toward: what was applied when it started
    self.applied = 0.0  # toward: what the last frame applied (after Smooth Turn Start)
    self.steady = deque(maxlen=HOLD_STEADY_FRAMES)  # Hold Ignores Crawl Spikes: the last 0.3 s of the hold's input

  def _clear_log(self, desired_curvature: float) -> None:
    """Per-frame record of what Turn Assist did, for fmControls.turnAssist. Nothing here feeds back into control."""
    self.log_source = SOURCE_NONE
    self.log_in = self.log_out = float(desired_curvature)
    self.log_driver = self.log_path = self.log_floor = 0.0
    self.log_nudge = self.log_smooth_cut = self.log_exit = self.log_floor_level = 0.0
    self.log_early = False

  @property
  def enabled(self) -> bool:
    return self.hold_enabled

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
    """Turn Nudge (always on with Turn Hold): a deliberate nudge into the turn adds a little, briefly."""
    stopping = a_ego < LEAD_DECEL_GATE and v_ego ** 2 / (-2.0 * a_ego) < HOLD_PATH_LOOKAHEAD[0]
    hand_torque = sign * steering_torque
    if (not self.hold_enabled or not lat_active or lane_changing or stopping
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
    self._clear_log(desired_curvature)
    # Finish Turn After Signal Cancels (plan PR 12) replaces this sign with the carried turn's once the desire ends
    sign = turn_sign(turn_direction) if self.enabled else 0
    self.log_source = SOURCE_DESIRE if sign != 0 else SOURCE_NONE
    step = DT_CTRL / FADE_TIME
    if sign == 0 and self.hold_exit_amount > 0.0:
      self.turn_ended = True
      # Only carry the last applied hold into the exit. Do not update its peak.
      self.amount = max(self.amount - step, 0.0)
      self.hold_exit_amount = max(self.hold_exit_amount - step, 0.0)
      driver_opposes = steering_pressed and not (self.hold_driver and self.sign * steering_torque < 0.0)
      if (self.hold_enabled and lat_active and not driver_opposes and not lane_changing
          and v_ego < MAX_SPEED and self.hold_exit_amount > 0.0):
        toward = self.sign * desired_curvature
        correction = max(self.hold_exit_curvature - toward, 0.0)
        self.log_exit = correction * self.hold_exit_amount
        self.log_out = desired_curvature + self.sign * self.log_exit
        return self.log_out
      self._reset_turn(0)
      return desired_curvature

    if sign != self.sign or self.turn_ended:
      self._reset_turn(sign)
      self.restarts = 0
    elif (self.restart_after_stop and sign != 0 and not lat_active and v_ego < HOLD_MOVING_SPEED
          and self.turned < HOLD_START_TURNED
          and (self.peak > 0.0 or self.hold_released or self.turned > 0.0)):
      # Stop Before Turning Starts Over: stopped before the truck has turned 10 degrees (a queue, a creep to the line):
      # nothing learned on the way (hold peak, a release) carries into the turn. Turn Lead (PR 11) adds its own state here.
      self._reset_turn(sign)
      self.restarts = min(self.restarts + 1, 255)
    if sign != 0:
      self.turned += sign * actual_curvature * v_ego * DT_CTRL
      self.peak_actual = max(self.peak_actual, sign * actual_curvature)

    # the speed fade is only worked out during a turn: this line runs on every control step of every drive
    target = float(np.interp(v_ego, [FULL_SPEED, MAX_SPEED], [1.0, 0.0])) if (sign != 0 and lat_active) else 0.0
    self.amount = step_toward(self.amount, target, step)
    self.hold_exit_amount = 0.0
    if self.amount <= 0.0 or sign == 0:
      self.extra = 0.0
      return desired_curvature

    out = desired_curvature
    toward = sign * out  # positive when the request turns the turn's way

    # Turn Lead (plan PR 11) goes here, before Turn Hold, so Turn Hold holds whatever it asked for

    before_hold = out
    model_toward = sign * desired_curvature
    # Hold Only Counts While Steering: with lateral off the request coming in is the truck's own curvature, not a
    # request, so the hold neither learns from it nor adds to it
    if self.hold_enabled and not self.hold_released and lat_active:
      if v_ego >= HOLD_MOVING_SPEED and self.turned >= HOLD_CLOCK_TURNED:
        self.turn_time += DT_CTRL  # time stopped (waiting to turn) and before turn-in doesn't count toward the 8 s
      self.heading += sign * actual_curvature * v_ego * DT_CTRL
      # Hand torque is positive to the LEFT and curvature positive to the RIGHT, so torque into the turn is sign * torque < 0
      # Steering into the turn never releases the hold (you often start turning the wheel before the truck turns);
      # the truck's curve is only adopted once it is actually turning that way.
      into_turn = steering_pressed and self.hold_driver and sign * steering_torque < 0.0
      driver_adds = into_turn and sign * actual_curvature > HOLD_DRIVER_MIN_CURVATURE
      against = steering_pressed and not into_turn
      self.against_time = self.against_time + DT_CTRL if against else 0.0
      if against:
        # Ignore Brief Hand Blips: a press against the turn only counts once it lasts or is firm
        against = self.against_time >= HOLD_AGAINST_TIME - 1e-6 or sign * steering_torque >= HOLD_AGAINST_TORQUE_FIRM
      pressing_against = steering_pressed and sign * steering_torque > 0.0
      early_correction = pressing_against and (
        (v_ego < HOLD_EARLY_MAX_SPEED and self.heading < HOLD_EARLY_MAX_HEADING and self.turn_time < HOLD_MAX_TIME)
        or (self.turned < HOLD_START_TURNED and self.against_time < HOLD_START_RELEASE_TIME))
      if early_correction:
        # Straightening before the turn: pass the request through, pause the fade, keep the peak; don't release
        self.heading = max(self.heading, 0.0)
        self.log_early = True
      elif against or self.heading >= HOLD_MAX_HEADING or self.turn_time >= HOLD_MAX_TIME:
        self.hold_released = True
        self.release_reason = (RELEASE_DRIVER if against else
                               RELEASE_TURNED if self.heading >= HOLD_MAX_HEADING else RELEASE_TIME)
      elif (self.hold_follow_exit and self.turned >= HOLD_EXIT_TURNED and not into_turn and self.peak > 0.0
            and model_toward < HOLD_EXIT_FRAC * self.peak):
        # the model is letting go of the turn: fade the hold out from what was applied, instead of keeping the floor
        self.hold_released = True
        self.release_reason = RELEASE_MODEL_EXIT
        self.exit_fade, self.exit_fade_curvature = 1.0, self.applied
      else:
        if driver_adds:
          wound = min(sign * actual_curvature, HOLD_DRIVER_MAX_CURVATURE)  # your curve, not the slider's
          if wound > toward:
            out = sign * (toward + (wound - toward) * self.amount)
            self.log_driver = sign * out - toward
            toward = sign * out
        if self.hold_path and model_v2 is not None and v_ego < HOLD_PATH_MAX_SPEED:
          path = self.hold_path_curvature(model_v2, sign)
          if path > toward:
            out = sign * (toward + (path - toward) * self.amount)
            self.log_path = sign * out - toward
            toward = sign * out
        # Hold Ignores Crawl Spikes: under 5 mph the peak only learns what has been asked for 0.3 s
        self.steady.append(toward)
        learn = min(self.steady) if v_ego < HOLD_STEADY_MAX_SPEED else toward
        if learn >= self.peak:
          self.peak, self.since_peak = learn, 0.0
        else:
          self.since_peak += DT_CTRL
        floor = self.peak * max(HOLD_RATIO - HOLD_DECAY * self.since_peak, 0.0)
        self.log_floor_level = floor
        if toward < floor:
          out = sign * (toward + (floor - toward) * self.amount)
          self.log_floor = sign * out - toward

    if self.exit_fade > 0.0:
      self.exit_fade = max(self.exit_fade - step, 0.0)
      correction = max(self.exit_fade_curvature - toward, 0.0) * self.exit_fade
      if lat_active and correction > 0.0:
        out = sign * (toward + correction)
        self.log_exit = correction
        toward = sign * out

    if lat_active and sign * (out - before_hold) > 0.0:
      self.hold_exit_curvature = sign * out
      self.hold_exit_amount = 1.0 if self.exit_fade <= 0.0 else self.exit_fade  # a signal cancel continues this fade
    # The timing nudge never feeds the hold peak or its signal-cancel fade.
    before_nudge = out
    out = self.turn_nudge(out, sign, v_ego, a_ego, model_v2, steering_pressed, steering_torque, lat_active, lane_changing)
    self.log_nudge = sign * (out - before_nudge)
    self.log_out = self._smooth_start(desired_curvature, out, sign)
    return self.log_out

  def fill_log(self, msg) -> None:
    """Write this frame's record into fmControls.turnAssist. msg is a fresh message, so a field at its default (0,
    False, none) is left alone: with no turn only the request in and out are written, 2 fields instead of 20, on every
    control step. Turn Lead's fields (lead, leadYield) stay 0 until plan PR 11."""
    if self.sign == 0 and self.log_source == SOURCE_NONE and self.amount == 0.0 and self.log_exit == 0.0:
      # no turn and nothing fading: _reset_turn(0) and _clear_log() left every other field at 0
      if self.log_in:
        msg.requestIn = float(self.log_in)
      if self.log_out:
        msg.requestOut = float(self.log_out)
      return
    for name, value in (
      ("sign", int(self.sign if (self.log_source != SOURCE_NONE or self.log_exit > 0.0) else 0)),
      ("source", int(self.log_source)),
      ("amount", float(self.amount)),
      ("requestIn", float(self.log_in)),
      ("requestOut", float(self.log_out)),
      ("driverAdds", float(self.log_driver)),
      ("holdPath", float(self.log_path)),
      ("holdFloor", float(self.log_floor)),
      ("nudge", float(self.log_nudge)),
      ("smoothCut", float(self.log_smooth_cut)),
      ("exitFade", float(self.log_exit)),
      ("holdActive", bool(self.hold_enabled and self.sign != 0 and not self.hold_released and self.log_source != SOURCE_NONE)),
      ("holdPeak", float(self.peak)),
      ("holdFloorLevel", float(self.log_floor_level)),
      ("turned", float(math.degrees(self.turned))),
      ("holdTurned", float(math.degrees(self.heading))),
      ("holdTime", float(self.turn_time)),
      ("releaseReason", int(self.release_reason)),
      ("earlyCorrection", bool(self.log_early)),
      ("againstTime", float(self.against_time)),
      ("restarts", int(self.restarts)),
    ):
      if value:
        setattr(msg, name, value)

  def _smooth_start(self, desired_curvature: float, out: float, sign: int) -> float:
    """Smooth Turn Start: the extra request toward the turn rises through a low-pass and drops at once."""
    target = sign * (out - desired_curvature)
    if self.phase_in and target > self.extra:
      self.extra += (target - self.extra) * PHASE_IN_ALPHA
    else:
      self.extra = target
    self.log_smooth_cut = max(target - self.extra, 0.0)
    self.applied = sign * desired_curvature + self.extra
    if self.hold_exit_amount > 0.0:
      # a blinker cancel fades out from what was actually applied, never from more
      self.hold_exit_curvature = min(self.hold_exit_curvature, sign * desired_curvature + self.extra)
    return desired_curvature + sign * self.extra
