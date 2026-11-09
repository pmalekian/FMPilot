"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import pyray as rl

from openpilot.selfdrive.ui.mici.layouts.home import MiciHomeLayout
from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
from openpilot.selfdrive.ui.sunnypilot.onroad import fm_jetlink_icon
from openpilot.system.ui.lib.application import FontWeight
from openpilot.system.ui.widgets.label import UnifiedLabel


class MiciHomeLayoutSP(MiciHomeLayout):
  def __init__(self):
    super().__init__()
    self._openpilot_label = UnifiedLabel("AdventurePilot", font_size=60, font_weight=FontWeight.DISPLAY, max_width=480, wrap_text=False)

  def _set_chestnut_visibility(self):
    usb_connected = ui_state.usb_connected
    usb_unknown = ui_state.usb_unknown
    chestnut_state = ui_state.chestnut_state
    loading = chestnut_state == ChestnutState.LOADING
    # FM Pilot (from zoompilot): Jetlink's big model ready but waiting for a switch: steady and dimmed
    waiting = chestnut_state == ChestnutState.WAITING

    # the port is known while Jetlink holds it
    self._usb_icon.set_visible(usb_connected and usb_unknown and not ui_state.adb_blocked)
    self._chestnut_loading_icon.set_opacity(0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0)))
    self._chestnut_loading_icon.set_visible(not usb_unknown and loading)
    self._chestnut_icon.set_opacity(fm_jetlink_icon.DIMMED if waiting else 1.0)
    self._chestnut_icon.set_visible(not usb_unknown and not loading and
                                    chestnut_state in (ChestnutState.READY, ChestnutState.ACTIVE, ChestnutState.WAITING))
    self._chestnut_failed_icon.set_visible(not usb_unknown and chestnut_state in (ChestnutState.UNCOMPILED, ChestnutState.FAILED))
