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
when openpilot isn't steering, and at once when you steer. Logged in modelDataV2SP (fmTurnSettle, fmTurnSettleOffset).
"""
import math

import numpy as np

PARAM = "FM_TurnSettle"
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


def _at(xs, ys, x):
  return float(np.interp(x, xs, ys))


class TurnSettle:
  def __init__(self):
    self.sign = 0  # turn being watched or settled after: +1 right, -1 left
    self.turned = 0.0  # rad toward it while its desire was on
    self.time_left = 0.0  # s of settling left
    self.curvature = 0.0  # 1/m added this frame (positive right)
    self.offset = 0.0  # m: lane center minus the plan at the look-ahead (positive right); 0 when nothing is seen
    self.active = False

  def update(self, enabled: bool, desire_sign: int, lane_changing: bool, v_ego: float, truck_curvature: float,
             lat_active: bool, steering_pressed: bool, blendful_trust: float,
             x_idxs, lines_y, line_probs, line_stds, edges_y, edge_stds, plan_x, plan_y) -> float:
    """Once per model frame: the curvature to add (positive right). lines_y: the 4 lane lines' y at x_idxs (index 1 =
    near left, 2 = near right); edges_y: the 2 road edges (0 left, 1 right); stds are the per-line values modelV2 logs;
    plan_x/plan_y: the planned path. y is positive to the right."""
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


def settle_from_model(settle: TurnSettle, enabled: bool, desire, lane_change_state, v_ego: float, sm, blendful_trust: float,
                      model_output: dict) -> float:
  """modeld_v2's call, once per model frame (after Blendful, whose trust it hands over to). desire: DH.desire."""
  from openpilot.cereal import log
  from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan
  if not enabled:
    return settle.update(False, 0, False, v_ego, 0.0, False, False, 0.0, None, None, None, None, None, None, None, None)
  sign = 1 if desire == log.Desire.turnRight else (-1 if desire == log.Desire.turnLeft else 0)
  lines = model_output['lane_lines'][0, :, :, 0]
  edges = model_output['road_edges'][0, :, :, 0] if 'road_edges' in model_output else np.zeros((2, lines.shape[1]))
  edge_stds = model_output['road_edges_stds'][0, :, 0, 0] if 'road_edges_stds' in model_output else (math.inf, math.inf)
  plan = model_output['plan'][0, :, Plan.POSITION]
  return settle.update(True, sign, lane_change_state != log.LaneChangeState.off, v_ego, float(sm['controlsState'].curvature),
                       bool(sm['carControl'].latActive), bool(sm['carState'].steeringPressed), float(blendful_trust),
                       np.asarray(ModelConstants.X_IDXS), lines, model_output['lane_lines_prob'][0, 1::2],
                       model_output['lane_lines_stds'][0, :, 0, 0], edges, edge_stds, plan[:, 0], plan[:, 1])
