"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

FM Pilot: Blendful's on-screen lane center line, from sunnypilot #2073 (commit 97135965e, royjr and contributors, MIT).
FM Pilot changes: only the blue center line is drawn (the numeric readout is left out so it doesn't sit on top of the
comma 4's other labels), it only draws while Blendful is the selected lane keeping mode, and lateral-active is checked
here rather than through a UIStateSP helper FM Pilot doesn't have.
"""
import numpy as np
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.sunnypilot.modeld_v2.blendful import LANE_WIDTH_CLIP
from openpilot.system.ui.lib.shader_polygon import draw_polygon

BLENDFUL_COLOR = (0, 200, 255)


def lateral_active() -> bool:
  sm = ui_state.sm
  if sm.valid["selfdriveStateSP"]:
    mads = sm["selfdriveStateSP"].mads
    if mads.available:
      from openpilot.selfdrive.ui.sunnypilot.ui_state import MADSState
      return mads.enabled and mads.state != MADSState.paused
  return ui_state.status in (UIStatus.ENGAGED, UIStatus.LAT_ONLY)


def center_line(left: np.ndarray, right: np.ndarray, stds, l_prob: float, r_prob: float, lane_width: float) -> np.ndarray:
  """Lane center the way Blendful sees it: each line weighted by how sure the model is of it."""
  half_width = np.clip(right[:, 1] - left[:, 1], *LANE_WIDTH_CLIP) / 2 if l_prob * r_prob > 0.5 else lane_width / 2
  l_var = stds[1] ** 2 / max(l_prob, 1e-4)
  r_var = stds[2] ** 2 / max(r_prob, 1e-4)
  center = left.copy()
  center[:, 1] = (r_var * (left[:, 1] + half_width) + l_var * (right[:, 1] - half_width)) / (l_var + r_var)
  return center


class BlendfulRenderer:
  def __init__(self, rect_relative: bool = False):
    self._rect_relative = rect_relative
    self._center_points = np.empty((0, 2), dtype=np.float32)
    self._trust = 0.0
    self._alpha = FirstOrderFilter(0.0, 0.3, DT_MDL)

  def update(self, renderer, max_idx: int, *map_args) -> None:
    sm = ui_state.sm
    if not ui_state.blendful_enabled or not sm.seen['modelDataV2SP']:
      self._alpha.x = 0.0
      self._center_points = np.empty((0, 2), dtype=np.float32)
      return
    blendful = sm['modelDataV2SP']
    if sm.updated['modelDataV2SP']:
      self._alpha.update(float(blendful.blendfulTrust > 0.0))
      if blendful.blendfulTrust > 0.0:
        self._trust = blendful.blendfulTrust

    left, right = renderer._lane_lines[1].raw_points, renderer._lane_lines[2].raw_points
    stds = list(sm['modelV2'].laneLineStds)
    if self._alpha.x < 0.01 or len(left) == 0 or len(right) != len(left) or len(stds) < 3:
      self._center_points = np.empty((0, 2), dtype=np.float32)
      return

    l_prob, r_prob = blendful.blendfulLeftProb, blendful.blendfulRightProb
    if blendful.blendfulTrust > 0.0 and l_prob + r_prob > 0.0:
      center = center_line(left, right, stds, l_prob, r_prob, blendful.blendfulLaneWidth)
      self._center_points = renderer._map_line_to_polygon(center, 0.15, 0.0, max_idx, *map_args)

  def draw(self, rect: rl.Rectangle) -> None:
    if not ui_state.blendful_enabled or not lateral_active():
      return
    if self._center_points.size and self._trust > 0.05:
      points = self._center_points + (rect.x, rect.y) if self._rect_relative else self._center_points
      alpha = int(np.clip(self._trust, 0.2, 1.0) * 255 * self._alpha.x)
      draw_polygon(rect, points, rl.Color(*BLENDFUL_COLOR, alpha))
