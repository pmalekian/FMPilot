"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

FM Pilot: Accel Boost bar on the comma 4's side panel, from sunnypilot #2090 (rav4kumar and Jason Wen, Nov 4 2026, MIT;
not yet merged there). FM Pilot changes: it shows for every boost mode, Gas, Predictive and Set Speed
(FM_AccelBoostMode), reads what the boost actually added from longitudinalPlanSP.fmAccelBoost, scales to that mode's
maximum, and has its own setting, FM_AccelBoostIndicator (off by default). It takes the confidence ball's place only
while it is shown.

Set Speed lifts the model's target exactly up to the cruise ramp, so the planner's min() ties the two and labels the
source "cruise" (the cruise candidate is listed first). In Set Speed mode a cruise source still counts as the boost
setting the speed; only a lead (the MPC) empties the bar. Without this the bar read empty for most of each Set Speed
boost (97% of boosting frames in a steady replay of set_speed_assist.py through the planner's min()).
"""
import math
import pyray as rl

from openpilot.cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.fm_boost_mode import MODE_OFF, MODE_SET_SPEED, boost_max
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

BAR_WIDTH = 26
TRACK_TOP_RATIO = 0.08
TRACK_BOTTOM_RATIO = 0.92
CAP_BLEND_START = 0.45
MAX_BOOST_BLEND = 0.55
FADE_THRESHOLD = 0.01

COLOR_TRACK = rl.Color(255, 255, 255, 28)
COLOR_FILL = rl.Color(0, 230, 120, 255)


def get_boost_fraction(boost: float, max_boost: float) -> float:
  if max_boost <= 0.0:
    return 0.0
  return min(max(boost / max_boost, 0.0), 1.0)


def lighten(color: rl.Color, amount: float) -> rl.Color:
  amount = min(max(amount, 0.0), 1.0)
  return rl.Color(int(color.r + (255 - color.r) * amount),
                  int(color.g + (255 - color.g) * amount),
                  int(color.b + (255 - color.b) * amount), color.a)


def get_boost_color(fraction: float) -> rl.Color:
  blend = max(0.0, (fraction - CAP_BLEND_START) / (1.0 - CAP_BLEND_START))
  return lighten(COLOR_FILL, MAX_BOOST_BLEND * blend)


PlanSource = log.LongitudinalPlan.LongitudinalPlanSource


def boost_sets_speed(source, mode: int) -> bool:
  """The boosted model target is what sets the speed: the model (e2e), or in Set Speed mode also the cruise ramp it was
  lifted to (a tie in the planner's min())."""
  return source == PlanSource.e2e or (mode == MODE_SET_SPEED and source == PlanSource.cruise)


def indicator_wanted() -> bool:
  """Setting on, a boost mode selected, openpilot controlling speed."""
  return bool(ui_state.fm_accel_boost_indicator and ui_state.fm_accel_boost_mode != MODE_OFF and ui_state.has_longitudinal_control)


class AccelBoostIndicator(Widget):
  def __init__(self, panel_width: float = SIDE_PANEL_WIDTH, bar_width: float = BAR_WIDTH):
    super().__init__()
    self._panel_width = panel_width
    self._bar_width = bar_width
    fps = max(gui_app.target_fps, 1)
    self._boost_filter = FirstOrderFilter(0.0, 0.15, 1 / fps)
    self._alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / fps)
    self._max_boost_filter = FirstOrderFilter(0.2, 0.25, 1 / fps)

  @property
  def holding_panel(self) -> bool:
    return self._alpha_filter.x > FADE_THRESHOLD

  def _update_state(self):
    sm = ui_state.sm
    visible = (indicator_wanted() and sm.alive['selfdriveState'] and sm.alive['longitudinalPlan'] and
               sm['selfdriveState'].enabled)
    # Only counts while the boosted model target is what sets the speed; if the lead (or, except for Set Speed, the
    # cruise ramp) is limiting, the boost isn't doing anything
    applied = (visible and sm.alive['longitudinalPlanSP'] and
               boost_sets_speed(sm['longitudinalPlan'].longitudinalPlanSource, ui_state.fm_accel_boost_mode))
    self._boost_filter.update(float(sm['longitudinalPlanSP'].fmAccelBoost) if applied else 0.0)
    self._alpha_filter.update(1.0 if visible else 0.0)

    if sm.alive['selfdriveState']:
      self._max_boost_filter.update(boost_max(ui_state.fm_accel_boost_mode, sm['selfdriveState'].personality))

  def _render(self, _):
    alpha = self._alpha_filter.x
    if alpha < FADE_THRESHOLD:
      return

    panel_x = max(self.rect.x, self.rect.x + self.rect.width - self._panel_width)
    bar_x = panel_x + (self._panel_width - self._bar_width) / 2
    track_top = self.rect.y + self.rect.height * TRACK_TOP_RATIO
    track = rl.Rectangle(bar_x, track_top, self._bar_width,
                         max(1.0, self.rect.height * (TRACK_BOTTOM_RATIO - TRACK_TOP_RATIO)))
    rl.draw_rectangle_rounded(track, 0.5, 12,
                              rl.Color(COLOR_TRACK.r, COLOR_TRACK.g, COLOR_TRACK.b, int(COLOR_TRACK.a * alpha)))

    fraction = get_boost_fraction(self._boost_filter.x, self._max_boost_filter.x)
    fill_h = fraction * track.height
    if fill_h < 1.0:
      return

    fill_color = get_boost_color(fraction)
    tip = track.y + track.height - fill_h
    rl.begin_scissor_mode(int(track.x), int(tip), int(math.ceil(track.width)) + 1, int(math.ceil(fill_h)))
    rl.draw_rectangle_rounded(track, 0.5, 12, rl.Color(fill_color.r, fill_color.g, fill_color.b, int(235 * alpha)))
    rl.end_scissor_mode()
