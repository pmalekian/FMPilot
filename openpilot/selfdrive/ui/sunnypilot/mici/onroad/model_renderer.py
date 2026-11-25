"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl
from openpilot.selfdrive.ui.ui_state import UIStatus
from openpilot.selfdrive.ui.sunnypilot.onroad.blendful import BlendfulRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.rainbow_path import RainbowPath

LANE_LINE_COLORS_SP = {
  UIStatus.LAT_ONLY: rl.Color(0, 255, 64, 255),
  UIStatus.LONG_ONLY: rl.Color(0, 255, 64, 255),
}


class ModelRendererSP:
  def __init__(self):
    self.rainbow_path = RainbowPath()
    # FM Pilot: Blendful's blue center line only; its readout would sit on top of the comma 4's other labels
    self.blendful = BlendfulRenderer(readout=False, rect_relative=True)
