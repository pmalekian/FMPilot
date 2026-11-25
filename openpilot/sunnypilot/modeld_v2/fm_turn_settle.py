"""FM Pilot: Settle After Turn (FM_TurnSettle).

After a blinker turn the truck often keeps whatever lane position the turn left it in: lane-line confidence stays near
zero for 3-5 s after a turn, and Blendful (and lane centering) wait for confident lines, above 5 m/s, with no desire. In
000000af (Nov 20) the truck sat 0.4-0.6 m left of center, the dashed line under its left half, for ~4 s after a right
turn (--1 +8..+12) and 0.6-0.9 m left after a left (--3 +38..+41). A lane line is often seen at 30-35% probability
1-2.5 s before Blendful takes over.

For up to 6 s after a blinker turn of at least 45 degrees ends (the turn desire goes away), from 8 to 30 mph, this adds a
small curvature toward the center of the lane, from what can be seen early:
  - lane lines at 30% probability or more (std under 0.9 m), where Blendful waits for confident ones;
  - a lane is the space between the two lines when both are seen (2.6-5 m apart), else 1.6 m in from the one that is;
  - a confident road edge (std under 0.5 m) only limits the aim (never closer than 1.4 m to it), it is never a target:
    replayed on 000000af, aiming 1.8 m in from the curb would have pushed a good right turn (--1 +39) 0.7 m toward a
    shoulder;
  - closer to the center than 0.25 m is left alone.
After a left turn it only ever pushes right, away from the left line (center line or median), never toward it. After a
right turn it centers both ways.

Strength: the lane is measured 6 m ahead (lines farther out swing where a turn lane opens); the curvature that moves
the truck that far over within 1.5 s of travel (at least 10 m), times 0.6, capped
at 0.004 1/m (0.18 m/s^2 at 15 mph), building and releasing at Blendful's rates (0.5 and 2 m/s^3 lateral). It stops
(and fades) once Blendful is confident (trust 0.5), at a lane change or a new turn desire, above 30 mph, after 6 s,
when openpilot isn't steering, and at once when you steer. Logged in fmModel (turnSettle, turnSettleOffset).

Finish Turn After Signal Cancels (fm-dev-sync): when Turn Assist carries the turn on after its desire ends (the blinker
cancelled at turn-in while the truck still turns, fmControls.turnAssist.source = carry), the carry is part of the turn
here too: the degrees turned keep counting through it and the end of the carry is the end of the turn, so the window
opens once the truck has straightened out of the carried turn (not at the blinker cancel, under 45 degrees turned, where
it never opened before). Exactly as if the turn desire had stayed on for the carry. controlsd's carry can reach modeld a
frame after modeld saw the desire end, so a carry of the same turn within 3 model frames (0.15 s) of that end takes it
back (the turned count is restored, a window it opened is undone). With no carry the result is the same as before.

Early Right Push (FM_TurnSettleEarlyPush, a second option under Settle After Turn, off; Build C of the Nov 23 build note).
With it off, everything above runs exactly as before (test_fm_turn_settle_early.py runs the code before this option on the
same random inputs and finds no difference). With it on:
  1. Starts as soon as the truck straightens, not only when the turn desire ends: after 45 deg or more turned, once the
     wheel's curvature has been under 30% of the turn's tightest and under 0.02 1/m (a 50 m radius) for 0.3 s and the
     model's plan bends no more than 0.5 m toward the turn 10 m ahead. The desire ending still starts it as before.
  2. From 3 mph (instead of 8) to 30 mph.
  3. Capped by lateral acceleration at low speed: 0.1 m/s^2, never more than 0.01 1/m, and never less than the old
     0.004 1/m (so above ~11 mph the cap is the old one).
  4. With no lane center to aim at (no line at 30%), a blind push to the right after left and right turns alike (US
     right-hand traffic: oncoming traffic is always to the left), at most 0.4 m in total, measured from the truck's own
     motion (wheel curvature and speed, from where the window started, so any drift right counts toward it), against the
     road's direction: the wheel's heading, slowly (1.5 s) pulled toward the direction of the model's plan 20 m (or 3 s)
     ahead, at most 2.9 deg from the truck's. The truck is only roughly lined up when the window starts, and 1 deg off
     would read as 1.7 cm of drift per meter.
     It aims at that point the way the centering aims at a lane center, so it eases off as the truck heads there.
  5. A 30%+ line that shows the truck centered (within the 0.25 m deadband) or right of center ends the push for this
     turn (good turns like 000000af Turns 2 and 4).
  6. Once a center is seen, the centering above takes over (after a left turn still never left), and Blendful still
     takes over at trust 0.5.
  7. Curb guard, whatever FM_LaneKeepMode is: a sure curb (std 0.5 m or less) stops the push 1.4 m from it, and when the
     planned path plus Settle's own curvature brings the body within Lane Guard's margin of it, steers back left with
     Lane Guard's math (get_lane_guard_curvature, gerrylum rx-wb 17ea338, MIT; Rivian half width 1.007 m, edge margin
     0.15 m), without Lane Guard's own gates (70% lines, 8 m/s, curves or 35 mph).
  8. Yields to the model going around something: when the plan at the look-ahead (1.5 s, at least 10 m) heads left of the truck by more than a
     quarter of the model's own curve explains (against the road's direction, so the plan pointing back along the road
     once the push has angled the truck right doesn't count), the rightward part is scaled down and held (gone at 0.45 m, 0.2 m once
     yielding, back under 0.1 m), the same thresholds and hysteresis as Blendful's going-around yield (sunnypilot #2073,
     MIT). The truck itself is the reference here (no lane is seen yet), where Blendful uses the usual spot in the lane.
  9. Hands win: any steering press drops it at once and ends the window, as before.
  10. Logged in fmModel: turnSettlePush (m right of where the window started, the most reached), turnSettleLimit (what
     stopped or holds it), turnSettlePhase (push / centering / handedOff), turnSettleGuard (curb push-back, 1/m).
The 3 mph floor, 0.1 m/s^2 cap and 0.4 m push are PROVISIONAL: picked without a replay (the 000000af replay of all 8 turns
against the current Settle is still owed before driving it).

FM Pilot's own (Settle After Turn from 35908da, Early Right Push from Build C on that branch), ported from fm-dev-nav-settle
ca08aac9 unchanged except for the carry above and the log moving to fmModel. Borrowed: the curb guard is Lane Guard's
road-edge math (get_lane_guard_curvature, gerrylum rx-wb 17ea338, MIT) and the yield uses Blendful's going-around
thresholds and hysteresis (sunnypilot #2073, James Vecellio-Grant / Discountchubbs, MIT); Blendful's build and release
rates are used throughout.
"""
import math

import numpy as np

PARAM = "FM_TurnSettle"
PARAM_EARLY = "FM_TurnSettleEarlyPush"
DT = 0.05  # modeld's frame
MIN_TURN = math.radians(45.0)  # only after a real turn
WINDOW = 6.0  # s after the turn desire ends
MIN_SPEED = 8 * 0.44704  # m/s
MAX_SPEED = 30 * 0.44704
LINE_MIN_PROB = 0.3
LINE_MAX_STD = 0.9  # m
EDGE_MAX_STD = 0.5  # m
HALF_LANE = 1.6  # m: lane center from a single line (a 3.2 m lane; keeps the 2 m truck 0.6 m off the line)
EDGE_CLEARANCE = 1.4  # m: never aim closer than this to a confident road edge
DEADBAND = 0.25  # m: closer to the lane center than this is left alone
LANE_WIDTH = (2.6, 5.0)  # m: two lines this far apart are one lane
LOOKAHEAD_TIME = 1.5  # s
MIN_LOOKAHEAD = 10.0  # m
SEE_X = 6.0  # m: where the lane is measured (lines far ahead swing at a turn lane opening or a merge)
GAIN = 0.6
MAX_ERROR = 1.0  # m
MAX_CURVATURE = 0.004  # 1/m
HANDOVER_TRUST = 0.5  # Blendful's trust at which it takes over

# Early Right Push. PROVISIONAL (no replay yet): EARLY_MIN_SPEED, LOW_SPEED_LAT_ACCEL, MAX_CURVATURE_LOW, PUSH_MAX.
EARLY_MIN_SPEED = 3 * 0.44704  # m/s; top of the 2-3 mph range: Turn Hold's crawl spikes (000000af T5) were at 1.9-4 mph
LOW_SPEED_LAT_ACCEL = 0.1  # m/s^2; gentler than the 0.18 the old cap gives at 15 mph; equals the old cap at ~11 mph
MAX_CURVATURE_LOW = 0.01  # 1/m (100 m radius): ceiling below ~7 mph, where 0.1 m/s^2 alone would allow tight curves
PUSH_MAX = 0.4  # m; in a 3.3 m lane a centered 2.01 m truck has 0.65 m each side, so 0.25 m is left to the right line
STRAIGHT_FRACTION = 0.3  # of the turn's tightest wheel curvature
STRAIGHT_CURVATURE = 0.02  # 1/m (50 m radius)
STRAIGHT_PLAN_X = 10.0  # m
STRAIGHT_PLAN_BEND = 0.5  # m toward the turn at STRAIGHT_PLAN_X (a 100 m radius)
STRAIGHT_TIME = 0.3  # s
YIELD = (0.2, 0.45)  # m, Blendful's going-around thresholds (sunnypilot #2073)
ROAD_X = 20.0  # m: the plan point that gives the road's direction (at least; 3 s ahead when faster)
ROAD_TAU = 1.5  # s: the plan's direction corrects the wheel's heading this slowly (a swerve around a car barely moves it)
ROAD_MAX = 0.05  # rad (2.9 deg): the most the truck is taken to be off the road's direction from one plan
GUARD_JERK = 0.8  # m/s^3, Lane Guard's Rivian tune: how fast the curb push-back may build
CARRY_GRACE = 3  # model frames (0.15 s): a carry this soon after the desire ended takes the turn back

# turnSettleLimit / turnSettlePhase values (custom.capnp FmModel.SettleLimit / SettlePhase)
LIMIT_NONE, LIMIT_CAP, LIMIT_CENTER, LIMIT_CURB, LIMIT_YIELD, LIMIT_HANDS, LIMIT_BLENDFUL, LIMIT_TIMEOUT, LIMIT_SPEED, \
  LIMIT_OTHER = range(10)
PHASE_IDLE, PHASE_PUSH, PHASE_CENTERING, PHASE_HANDED_OFF = range(4)


def _at(xs, ys, x):
  return float(np.interp(x, xs, ys))


def low_speed_cap(v_ego: float) -> float:
  """Early Right Push's curvature cap: 0.1 m/s^2 of lateral accel, between the old 0.004 and 0.01 1/m."""
  return float(np.clip(LOW_SPEED_LAT_ACCEL / max(v_ego, 0.1) ** 2, MAX_CURVATURE, MAX_CURVATURE_LOW))


def curb_guard(v_ego: float, plan_x, plan_y, settle_curvature: float, x_idxs, right_edge_y) -> float:
  """Lane Guard's road-edge math (gerrylum rx-wb 17ea338, MIT; Rivian half width 1.007 m, edge margin 0.15 m) on the
  plan as Settle's own curvature bends it, against the right road edge only: 0 or a curvature to the left (negative)."""
  from openpilot.selfdrive.controls.lib.lane_guard import TUNES, get_lane_guard_curvature
  plan_x = np.asarray(plan_x, dtype=float)
  bent_y = np.asarray(plan_y, dtype=float) + settle_curvature * plan_x ** 2 / 2
  return min(get_lane_guard_curvature(v_ego, plan_x, bent_y, None, None, None, (np.asarray(x_idxs), np.asarray(right_edge_y)),
                                      tune=TUNES["rivian"]), 0.0)


class TurnSettle:
  def __init__(self):
    self.sign = 0  # turn being watched or settled after: +1 right, -1 left
    self.turned = 0.0  # rad toward it while its desire was on
    self.time_left = 0.0  # s of settling left
    self.curvature = 0.0  # 1/m added this frame (positive right)
    self.offset = 0.0  # m: lane center minus the plan at the look-ahead (positive right); 0 when nothing is seen
    self.active = False
    # Early Right Push only (the classic path never reads or writes these)
    self.early = False
    self._reset_early()
    # Finish Turn After Signal Cancels: the turn that just ended, for a carry that shows up a frame or two late
    self.ended = None  # (sign, turned, frames left, opened a window) or None

  def _reset_early(self):
    self.peak = 0.0  # 1/m: the turn's tightest wheel curvature toward it
    self.straight_time = 0.0  # s the truck has looked straightened
    self.prev_desire_sign = 0
    self.settled = False  # this turn's window has started (so the desire ending doesn't start another)
    self.psi = 0.0  # rad: the truck's heading since the window started, from its wheel curvature and speed (right +)
    self.road = None  # rad: the road's heading in that frame: the wheel's short term, corrected toward the plan's
    self.y = 0.0  # m: how far right of the road line it started on the truck has moved since
    self.push = 0.0  # m: the most it got right of where the window started
    self.push_done = False  # the push is over for this turn (cap, center or curb)
    self.yielding = False
    self.limit = LIMIT_NONE
    self.phase = PHASE_IDLE
    self.guard = 0.0  # 1/m: the curb push-back this frame

  def update(self, enabled: bool, desire_sign: int, lane_changing: bool, v_ego: float, truck_curvature: float,
             lat_active: bool, steering_pressed: bool, blendful_trust: float,
             x_idxs, lines_y, line_probs, line_stds, edges_y, edge_stds, plan_x, plan_y,
             early: bool = False, model_curvature: float = 0.0, carry_sign: int = 0) -> float:
    """Once per model frame: the curvature to add (positive right). lines_y: the 4 lane lines' y at x_idxs (index 1 =
    near left, 2 = near right); edges_y: the 2 road edges (0 left, 1 right); stds are the per-line values modelV2 logs;
    plan_x/plan_y: the planned path (modelV2's 33 points, at ModelConstants.T_IDXS). y is positive to the right. early: Early Right Push; model_curvature:
    the model's own request this frame (only the early push reads it, for the yield). carry_sign: the turn Turn Assist
    is carrying on after its desire ended (Finish Turn After Signal Cancels), 0 when none."""
    if early != self.early:  # switched while driving: end any window, keep the curvature fading at the usual rates
      self.early = early
      self.sign, self.turned, self.time_left, self.active = 0, 0.0, 0.0, False
      self._reset_early()
      self.ended = None
    desire_sign = self._turn_sign(enabled, desire_sign, carry_sign, early)
    if early and enabled:
      return self._update_early(desire_sign, lane_changing, v_ego, truck_curvature, lat_active, steering_pressed,
                                blendful_trust, x_idxs, lines_y, line_probs, line_stds, edges_y, edge_stds, plan_x, plan_y,
                                model_curvature)
    return self._update_classic(enabled, desire_sign, lane_changing, v_ego, truck_curvature, lat_active, steering_pressed,
                                blendful_trust, x_idxs, lines_y, line_probs, line_stds, edges_y, edge_stds, plan_x, plan_y)

  def _update_classic(self, enabled, desire_sign, lane_changing, v_ego, truck_curvature, lat_active, steering_pressed,
                      blendful_trust, x_idxs, lines_y, line_probs, line_stds, edges_y, edge_stds, plan_x, plan_y):
    """Settle After Turn as built Nov 20 (35908da), unchanged."""
    self.offset = 0.0
    if not enabled:
      self.__init__()
      return 0.0

    # follow the turn: which way, how far
    if desire_sign != 0:
      if desire_sign != self.sign or self.time_left > 0.0:
        self.sign, self.turned, self.time_left = desire_sign, 0.0, 0.0
      self.turned += desire_sign * truck_curvature * max(v_ego, 0.0) * DT
    elif self.sign != 0 and self.time_left <= 0.0 and not self.active:
      self.ended = (self.sign, self.turned, CARRY_GRACE, self.turned >= MIN_TURN)
      if self.turned >= MIN_TURN:
        self.time_left = WINDOW  # the turn just ended: settle
      else:
        self.sign = 0
      self.turned = 0.0

    stop = (desire_sign != 0 or lane_changing or not lat_active or steering_pressed or v_ego > MAX_SPEED
            or blendful_trust >= HANDOVER_TRUST)
    if self.time_left > 0.0:
      self.time_left -= DT
      if stop:
        self.time_left = 0.0
    self.active = self.time_left > 0.0 and v_ego >= MIN_SPEED

    target = 0.0
    if self.active:
      lookahead = max(v_ego * LOOKAHEAD_TIME, MIN_LOOKAHEAD)
      center = self._lane_center(SEE_X, x_idxs, lines_y, line_probs, line_stds, edges_y, edge_stds)
      if center is not None:
        self.offset = center - _at(plan_x, plan_y, SEE_X)
        error = float(np.clip(self.offset, -MAX_ERROR, MAX_ERROR))
        error = math.copysign(max(abs(error) - DEADBAND, 0.0), error)
        if self.sign < 0:
          error = max(error, 0.0)  # after a left: only away from the left line
        target = float(np.clip(GAIN * 2.0 * error / lookahead ** 2, -MAX_CURVATURE, MAX_CURVATURE))

    # Blendful's rates: build at 0.5 m/s^3 lateral jerk, let go 4x faster (at once when you steer)
    v2 = max(v_ego, 1.0) ** 2
    build, release = 0.5 / v2 * DT, 2.0 / v2 * DT
    k = self.curvature
    if steering_pressed:
      k = 0.0
    else:
      growing = abs(target) > abs(k) and target * k >= 0.0
      rate = build if growing else release
      k += float(np.clip(target - k, -rate, rate))
    self.curvature = k
    if not self.active and self.curvature == 0.0 and self.time_left <= 0.0 and desire_sign == 0:
      self.sign = 0
    return self.curvature

  def _update_early(self, desire_sign, lane_changing, v_ego, truck_curvature, lat_active, steering_pressed, blendful_trust,
                    x_idxs, lines_y, line_probs, line_stds, edges_y, edge_stds, plan_x, plan_y, model_curvature):
    """Early Right Push (FM_TurnSettleEarlyPush): see the module docstring, items 1-10."""
    self.offset = 0.0
    self.guard = 0.0
    v = max(v_ego, 0.0)

    # a new turn: a desire appears, or flips side
    new_turn = desire_sign != 0 and desire_sign != self.prev_desire_sign
    self.prev_desire_sign = desire_sign
    ended = None
    if new_turn:  # ends any window still running for the last turn
      self.sign, self.turned, self.time_left = desire_sign, 0.0, 0.0
      self._reset_early()
      self.prev_desire_sign = desire_sign

    # 1. watch the turn; start once the truck has straightened (or, as before, once the desire ends)
    if self.sign != 0 and not self.settled:
      if desire_sign == self.sign:
        self.turned += desire_sign * truck_curvature * v * DT
        toward = desire_sign * truck_curvature
        self.peak = max(self.peak, toward)
        bend = desire_sign * _at(plan_x, plan_y, STRAIGHT_PLAN_X)
        straight = toward < min(STRAIGHT_FRACTION * self.peak, STRAIGHT_CURVATURE) and bend <= STRAIGHT_PLAN_BEND
        self.straight_time = self.straight_time + DT if straight else 0.0
        if self.turned >= MIN_TURN and self.straight_time >= STRAIGHT_TIME - 1e-9:
          self._start_window()
      elif desire_sign == 0:
        self.ended = (self.sign, self.turned, CARRY_GRACE, self.turned >= MIN_TURN)
        if self.turned >= MIN_TURN:
          self._start_window()
        else:
          self.sign = 0
          self.turned = 0.0

    # the truck's own motion since the window started (wheel curvature and speed), against the road's direction: the
    # truck is only roughly lined up when the window starts, and 1 deg off reads as 1.7 cm of drift per meter driven
    if self.time_left > 0.0:
      if v > 0.5 or self.road is None:
        far = min(max(ROAD_X, v * 3.0), float(plan_x[-1])) if plan_x[-1] > 1.0 else ROAD_X
        heading = float(np.clip(-math.atan2(_at(plan_x, plan_y, far), far), -ROAD_MAX, ROAD_MAX))  # truck vs road
        road = self.psi - heading
        self.road = road if self.road is None else self.road + (road - self.road) * DT / (ROAD_TAU + DT)
      self.psi += truck_curvature * v * DT
      self.y += v * math.sin(self.psi - self.road) * DT
      self.push = max(self.push, self.y)

    # what ends the window (a desire still on from the turn just finished doesn't)
    if self.time_left > 0.0:
      self.time_left -= DT
      if steering_pressed:
        ended = LIMIT_HANDS
      elif blendful_trust >= HANDOVER_TRUST:
        ended = LIMIT_BLENDFUL
      elif v > MAX_SPEED:
        ended = LIMIT_SPEED
      elif lane_changing or not lat_active:
        ended = LIMIT_OTHER
      elif self.time_left <= 0.0:
        ended = LIMIT_TIMEOUT
      if ended is not None:
        self.time_left = 0.0
    if ended is not None:
      self.limit = ended
      self.phase = PHASE_HANDED_OFF if ended == LIMIT_BLENDFUL else self.phase
    self.active = self.time_left > 0.0 and v >= EARLY_MIN_SPEED

    target = 0.0
    if self.active:
      lookahead = max(v * LOOKAHEAD_TIME, MIN_LOOKAHEAD)
      cap = low_speed_cap(v)
      center = self._lane_center(SEE_X, x_idxs, lines_y, line_probs, line_stds, edges_y, edge_stds)
      plan_see = _at(plan_x, plan_y, SEE_X)
      sure_curb = edge_stds[1] <= EDGE_MAX_STD
      if center is not None:
        # 5./6. a line is seen: centered or right of it ends the push for this turn; the centering takes over
        self.phase = PHASE_CENTERING
        self.offset = center - plan_see
        if self.offset <= DEADBAND and not self.push_done:
          self.push_done, self.limit = True, LIMIT_CENTER
        error = float(np.clip(self.offset, -MAX_ERROR, MAX_ERROR))
        error = math.copysign(max(abs(error) - DEADBAND, 0.0), error)
        if self.sign < 0:
          error = max(error, 0.0)  # after a left: only away from the left line
        target = float(np.clip(GAIN * 2.0 * error / lookahead ** 2, -cap, cap))
      else:
        # 4. blind push right, toward a point PUSH_MAX right of where the window started, aimed like the centering
        self.phase = PHASE_PUSH
        if not self.push_done and self.push >= PUSH_MAX:
          self.push_done, self.limit = True, LIMIT_CAP
        if not self.push_done:
          error = (PUSH_MAX - self.y) - (self.psi - (self.road or 0.0)) * lookahead
          if sure_curb:  # 7. a sure curb: stop the push 1.4 m from it
            room = _at(x_idxs, edges_y[1], SEE_X) - EDGE_CLEARANCE - plan_see
            if room <= 0.0:
              self.push_done, self.limit = True, LIMIT_CURB
            elif room < error:
              self.limit = LIMIT_CURB  # the curb is what holds it back now
            elif self.limit == LIMIT_CURB:
              self.limit = LIMIT_NONE
            error = min(error, room)
          if not self.push_done:
            target = float(np.clip(GAIN * 2.0 * error / lookahead ** 2, 0.0, cap))

      # 8. yield: the plan heads left of the truck more than a quarter of the model's own curve explains
      if target > 0.0:
        # against the road's direction: the plan pointing back along the road once the truck heads right of it (as the
        # push itself makes it) isn't going around anything
        away = -(_at(plan_x, plan_y, lookahead) + (self.psi - (self.road or 0.0)) * lookahead) \
               - 0.25 * abs(model_curvature) * lookahead ** 2 / 2
        if away > YIELD[0]:
          self.yielding = True
        elif away < YIELD[0] / 2:
          self.yielding = False
        scale = np.interp(away, (YIELD[0] / 2, YIELD[0]), [1.0, 0.0]) if self.yielding else np.interp(away, YIELD, [1.0, 0.0])
        target *= float(scale)
        if target == 0.0:
          self.limit = LIMIT_YIELD
        elif self.limit == LIMIT_YIELD:
          self.limit = LIMIT_NONE

      # 7. curb guard: the plan, bent by Settle's own curvature, brings the body within Lane Guard's margin: steer back
      if sure_curb:
        self.guard = curb_guard(v, plan_x, plan_y, self.curvature, x_idxs, edges_y[1])
        if self.guard < 0.0:
          target = min(target, self.guard)
          if self.phase == PHASE_PUSH and not self.push_done:
            self.push_done, self.limit = True, LIMIT_CURB

    # Blendful's rates (Lane Guard's 0.8 m/s^3 for the curb push-back); hands drop it at once
    v2 = max(v, 1.0) ** 2
    build, release = 0.5 / v2 * DT, 2.0 / v2 * DT
    k = self.curvature
    if steering_pressed:
      k = 0.0
    else:
      growing = abs(target) > abs(k) and target * k >= 0.0
      rate = build if growing else release
      if self.guard < 0.0:
        rate = max(rate, GUARD_JERK / v2 * DT)
      k += float(np.clip(target - k, -rate, rate))
    self.curvature = k
    if not self.active and self.time_left <= 0.0 and self.phase != PHASE_HANDED_OFF:
      self.phase = PHASE_IDLE
    if self.phase == PHASE_HANDED_OFF and self.curvature == 0.0 and ended is None:  # logged at least the handover frame
      self.phase = PHASE_IDLE
    if not self.active and self.curvature == 0.0 and self.time_left <= 0.0 and desire_sign == 0 and self.settled:
      self.sign = 0
      self.settled = False
    return self.curvature

  def _turn_sign(self, enabled: bool, desire_sign: int, carry_sign: int, early: bool) -> int:
    """Finish Turn After Signal Cancels: the turn sign to follow this frame. A carry of the turn being followed counts
    as its desire still being on; a carry of the turn that ended up to CARRY_GRACE frames ago takes that turn back
    first. Any other carry is ignored, and with no carry this is the desire, so everything runs as before."""
    ended, self.ended = self.ended, None
    if not enabled or desire_sign != 0:
      return desire_sign
    if carry_sign == 0:
      if ended is not None and ended[2] > 1:
        self.ended = (ended[0], ended[1], ended[2] - 1, ended[3])
      return 0
    if ended is not None and ended[0] == carry_sign:
      sign, turned, _, opened = ended
      self.sign, self.turned = sign, turned
      if opened:  # undo the window that turn's end opened
        self.time_left, self.active = 0.0, False
        if early:
          self.settled, self.phase, self.limit = False, PHASE_IDLE, LIMIT_NONE
      if early:
        self.prev_desire_sign = sign
      return carry_sign
    return carry_sign if self.sign == carry_sign else 0

  def _start_window(self):
    self.time_left = WINDOW
    self.settled = True
    self.turned = 0.0
    self.y = self.psi = self.push = 0.0
    self.road = None
    self.push_done, self.yielding, self.limit = False, False, LIMIT_NONE

  def _lane_center(self, x, x_idxs, lines_y, line_probs, line_stds, edges_y, edge_stds):
    left = _at(x_idxs, lines_y[1], x) if line_probs[1] >= LINE_MIN_PROB and line_stds[1] <= LINE_MAX_STD else None
    right = _at(x_idxs, lines_y[2], x) if line_probs[2] >= LINE_MIN_PROB and line_stds[2] <= LINE_MAX_STD else None
    if left is not None and right is not None:
      if LANE_WIDTH[0] <= right - left <= LANE_WIDTH[1]:
        center = (left + right) / 2.0
      else:  # not one lane: keep to the side the turn was toward
        center = right - HALF_LANE if self.sign > 0 else left + HALF_LANE
    elif left is not None:
      center = left + HALF_LANE
    elif right is not None:
      center = right - HALF_LANE
    else:
      return None
    # a confident road edge is a limit, never a target (a shoulder or parking lane puts the curb far from the lane)
    if edge_stds[1] <= EDGE_MAX_STD:
      center = min(center, _at(x_idxs, edges_y[1], x) - EDGE_CLEARANCE)
    if edge_stds[0] <= EDGE_MAX_STD:
      center = max(center, _at(x_idxs, edges_y[0], x) + EDGE_CLEARANCE)
    return center


def carry_from_sm(sm) -> int:
  """The turn Turn Assist is carrying (fmControls.turnAssist, source carry): +1 right, -1 left, 0 none."""
  seen = getattr(sm, 'seen', None)
  if not seen or not seen.get('fmControls', False):  # controlsd hasn't published (or an sm without it, in tests)
    return 0
  ta = sm['fmControls'].turnAssist
  return int(ta.sign) if str(ta.source) == 'carry' else 0


def settle_from_model(settle: TurnSettle, enabled: bool, desire, lane_change_state, v_ego: float, sm, blendful_trust: float,
                      model_output: dict, early: bool = False, model_curvature: float = 0.0) -> float:
  """modeld_v2's call, once per model frame (after Blendful, whose trust it hands over to). desire: DH.desire.
  early: FM_TurnSettleEarlyPush; model_curvature: the model's own request this frame (the early push's yield reads it).
  The carry comes from controlsd's fmControls in sm."""
  from openpilot.cereal import log
  from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan
  if not enabled:
    return settle.update(False, 0, False, v_ego, 0.0, False, False, 0.0, None, None, None, None, None, None, None, None,
                         early=early)
  sign = 1 if desire == log.Desire.turnRight else (-1 if desire == log.Desire.turnLeft else 0)
  lines = model_output['lane_lines'][0, :, :, 0]
  edges = model_output['road_edges'][0, :, :, 0] if 'road_edges' in model_output else np.zeros((2, lines.shape[1]))
  edge_stds = model_output['road_edges_stds'][0, :, 0, 0] if 'road_edges_stds' in model_output else (math.inf, math.inf)
  plan = model_output['plan'][0, :, Plan.POSITION]
  return settle.update(True, sign, lane_change_state != log.LaneChangeState.off, v_ego, float(sm['controlsState'].curvature),
                       bool(sm['carControl'].latActive), bool(sm['carState'].steeringPressed), float(blendful_trust),
                       np.asarray(ModelConstants.X_IDXS), lines, model_output['lane_lines_prob'][0, 1::2],
                       model_output['lane_lines_stds'][0, :, 0, 0], edges, edge_stds, plan[:, 0], plan[:, 1],
                       early=early, model_curvature=float(model_curvature), carry_sign=carry_from_sm(sm))
