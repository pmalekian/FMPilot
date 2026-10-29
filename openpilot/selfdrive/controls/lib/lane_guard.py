import math
from dataclasses import dataclass

import numpy as np

from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.ldw import CAMERA_OFFSET
from openpilot.selfdrive.modeld.constants import ModelConstants

LaneChangeState = log.LaneChangeState

# Adds a small curvature on top of the model's so the body stays inside the lane lines and never crosses a road edge.
# It stays at zero while the model's planned path keeps a margin from them.

MARGIN = 0.15  # m, gap to keep between the body and a road edge
LOOKAHEAD_T = (1.0, 1.5, 2.0)  # s, points of the plan that are checked, closer ones oscillate with the steering lag
MIN_LOOKAHEAD_DIST = 10.0  # m
MIN_SPEED = 8.0  # m/s
MIN_LINE_PROB, MIN_LINE_PROB_KEEP = 0.7, 0.6  # a line is trusted above the first, and stays trusted down to the second
MAX_LINE_STD = 0.3  # m
MAX_EDGE_STD = 1.0  # m
MAX_EDGE_T = 1.5  # s, the edge 2 s ahead often runs straight on past a junction the model turns into
MIN_LANE_WIDTH = 2.0  # m, narrower is a misdetection
MAX_LANE_WIDTH = 4.5  # m, wider is a missing center line, a merge, an exit or a shoulder: only the curb line is guarded
LANE_CHANGE_COOLDOWN = 1.0  # s, lines are unguarded this long after a lane change or the blinker
MIN_CURVE = 1 / 2000.  # 1/m, lines are guarded in curves, where the model cuts them
LANE_CURVE_T = 2.5  # s, how far ahead the lane curvature is measured


@dataclass(frozen=True)
class Tune:
  half_width: float = 1.921 / 2  # m, Model Y body without mirrors
  max_lat_accel: float = 1.0  # m/s^2
  max_lat_jerk: float = 4.0  # m/s^3
  target_tau: float = 0.0  # s, low-pass on the target, 0 is off
  line_margin: float = MARGIN  # m, gap to keep from a lane line
  # On straights the model leaves the lane on purpose, to pass parked cars or make room in a bike lane, so lines are only
  # guarded in curves. Above this speed those are rare and lines are guarded on straights too.
  straight_min_speed: float = math.inf


STOCK = Tune()
TUNES = {
  # R1T/R1S body is 2.014 m. Gentle and low-passed: on route 00000014 74 noticeable nudges became 21, p95 lateral jerk
  # 4 -> 0.5 m/s^3. The model holds the truck 10-35 cm right of center, so lines are guarded on straights above 35 mph.
  "rivian": Tune(half_width=2.014 / 2, max_lat_accel=0.4, max_lat_jerk=0.8, target_tau=0.5, line_margin=0.25, straight_min_speed=15.6),
}


def get_lane_curvature(v_ego, lines):
  # curvature of the mean of the lines, positive to the right like desiredCurvature
  xs = np.linspace(5.0, max(30.0, LANE_CURVE_T * v_ego), 10)
  center = np.mean([np.interp(xs, x, y) for x, y in lines], axis=0)
  return 2 * np.polyfit(xs, center, 2)[0]


def _push(gaps):
  # closer than the margin pushes away, and the part over the line counts twice
  return max((max(margin - gap, 0.0) + max(-gap, 0.0) for gap, margin in gaps), default=0.0)


def get_lane_guard_curvature(v_ego, path_x, path_y, left, right, left_edge, right_edge, left_hand_traffic=False, tune=STOCK):
  # left, right, left_edge, right_edge are (x, y), or None when not trusted. Without the curb line (right, left in
  # left-hand traffic) the other one is often only the imagined middle of the road, so lines need the curb line.
  curb = left if left_hand_traffic else right
  lines = [line for line in (left, right) if line is not None]
  guard_lines = curb is not None and (v_ego > tune.straight_min_speed or abs(get_lane_curvature(v_ego, lines)) > MIN_CURVE)

  curvature = 0.0
  for t in LOOKAHEAD_T:
    x = np.interp(t, ModelConstants.T_IDXS, path_x)
    car_y = np.interp(t, ModelConstants.T_IDXS, path_y) - CAMERA_OFFSET
    body_left, body_right = car_y - tune.half_width, car_y + tune.half_width
    y = {k: np.interp(x, *v) for k, v in (("left", left), ("right", right), ("left_edge", left_edge), ("right_edge", right_edge)) if v is not None}

    # (gap, margin) to everything guarded on each side
    near = t <= MAX_EDGE_T
    lefts = [(body_left - y["left_edge"], MARGIN)] if "left_edge" in y and near else []
    rights = [(y["right_edge"] - body_right, MARGIN)] if "right_edge" in y and near else []
    if guard_lines:
      lane_width = y["right"] - y["left"] if "left" in y and "right" in y else None
      if lane_width is None or lane_width >= MIN_LANE_WIDTH:
        wide = lane_width is not None and lane_width >= MAX_LANE_WIDTH  # the far line is often the other road edge
        if "left" in y and (left_hand_traffic or not wide):
          lefts.append((body_left - y["left"], tune.line_margin))
        if "right" in y and (not left_hand_traffic or not wide):
          rights.append((y["right"] - body_right, tune.line_margin))

    # pure pursuit: the curvature that moves the path by the push at distance x
    point = 2 / max(x, MIN_LOOKAHEAD_DIST) ** 2 * (_push(lefts) - _push(rights))
    if abs(point) > abs(curvature):
      curvature = point

  max_curvature = tune.max_lat_accel / max(v_ego, MIN_SPEED) ** 2
  return float(np.clip(curvature, -max_curvature, max_curvature))


class LaneGuard:
  def __init__(self, tune=STOCK, dt=DT_CTRL):
    self.tune, self.dt = tune, dt
    self.target = self.curvature = self.lane_change_timer = 0.0
    self.line_valid = [False, False]  # left, right

  def update(self, v_ego, model_v2, active, blinker=False, left_hand_traffic=False):
    # a blinker means the driver is about to leave the lane, also below the lane change speed. Road edges stay guarded.
    lane_change = blinker or model_v2.meta.laneChangeState != LaneChangeState.off
    self.lane_change_timer = LANE_CHANGE_COOLDOWN if lane_change else max(self.lane_change_timer - self.dt, 0.0)

    lane_lines, road_edges = model_v2.laneLines, model_v2.roadEdges
    full = len(lane_lines) == 4 and len(model_v2.laneLineStds) == 4
    for i, idx in enumerate((1, 2)):
      min_prob = MIN_LINE_PROB_KEEP if self.line_valid[i] else MIN_LINE_PROB
      self.line_valid[i] = self.lane_change_timer == 0.0 and full and \
                           model_v2.laneLineProbs[idx] > min_prob and model_v2.laneLineStds[idx] < MAX_LINE_STD

    target = 0.0
    if active and v_ego > MIN_SPEED and full and len(model_v2.position.x) == ModelConstants.IDX_N and \
       len(road_edges) == 2 and len(model_v2.roadEdgeStds) == 2:
      left, right = ((lane_lines[idx].x, lane_lines[idx].y) if self.line_valid[i] else None for i, idx in enumerate((1, 2)))
      left_edge, right_edge = ((e.x, e.y) if std < MAX_EDGE_STD else None for e, std in zip(road_edges, model_v2.roadEdgeStds, strict=True))
      target = get_lane_guard_curvature(v_ego, model_v2.position.x, model_v2.position.y, left, right, left_edge, right_edge,
                                        left_hand_traffic, self.tune)

    if self.tune.target_tau > 0.0:
      self.target += (target - self.target) * min(self.dt / self.tune.target_tau, 1.0)
      target = self.target
    max_change = self.tune.max_lat_jerk / max(v_ego, MIN_SPEED) ** 2 * self.dt
    self.curvature = float(np.clip(target, self.curvature - max_change, self.curvature + max_change))
    return self.curvature
