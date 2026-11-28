import math
import raylib_nb as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad import DIMMED_ICON_ALPHA, glow_highlight_texture
from openpilot.selfdrive.ui.mici.onroad.status import StatusFade, lerp_color
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.sunnypilot.onroad import fm_jetlink_icon
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus, ChestnutState
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.cereal import log
from openpilot.selfdrive.ui.sunnypilot.onroad.fm_torque_readout import (READOUT_CPU_AVG, READOUT_CPU_BUSIEST, READOUT_LEAD,
                                                                      READOUT_OFF, TURN_NOTICE_TYPES, LeadReadoutState,
                                                                      make_cpu_readout, make_readout)


# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

SET_SPEED_PERSISTENCE = 2.5  # seconds
WHEEL_SIZE = 52
WHEEL_WHITE = rl.Color(255, 255, 255, 255)
WHEEL_GREEN = rl.Color(0, 255, 64, 255)
WHEEL_ORANGE = rl.Color(255, 115, 0, 255)


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 36
  set_speed: int = 112


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)


FONT_SIZES = FontSizes()
COLORS = Colors()


FM_TURN_NOTICE_OFFSET = 90  # leave room for the turn notice headline
FM_READOUT_PERIOD = 0.1  # s, about 10 updates a second so the numbers are readable
FM_READOUT_COLORS = {
  'normal': rl.Color(255, 255, 255, 230),
  'at_cap': rl.Color(255, 176, 0, 255),
  'refused': rl.Color(255, 64, 64, 255),
  'off': rl.Color(160, 160, 160, 200),
}
FM_CHECK_COLOR = rl.Color(70, 220, 90, 255)
FM_LEAD_COLORS = {  # Lead readout (fm_torque_readout.py)
  'white': rl.Color(255, 255, 255, 230),
  'gray': rl.Color(160, 160, 160, 220),
  'cyan': rl.Color(110, 225, 255, 255),    # light cyan: camera-only lead under 50%
  'purple': rl.Color(205, 170, 255, 255),  # light lavender: radar-matched lead under 50%
  'red': rl.Color(255, 64, 64, 255),       # X: ghost dropped
  'red_dim': rl.Color(255, 64, 64, 70),
}


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self._set_speed_changed_time: float = 0
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self._engaged: bool = False
    self._chestnut_fade_time: float = 0

    self._can_draw_top_icons = True
    self._show_wheel_critical = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)

    self._torque_bar = TorqueBar()

    self._txt_wheel: rl.Texture = gui_app.texture('icons_mici/wheel.png', WHEEL_SIZE, WHEEL_SIZE)
    self._txt_wheel_highlight = glow_highlight_texture('icons_mici/wheel.png', WHEEL_SIZE)
    self._txt_chestnut: rl.Texture = gui_app.texture('icons_mici/chestnut.png', 48, 36)
    self._txt_chestnut_green: rl.Texture = gui_app.texture('icons_mici/chestnut_green.png', 48, 36)
    self._txt_chestnut_orange: rl.Texture = gui_app.texture('icons_mici/chestnut_orange.png', 61, 36)
    self._chestnut_icon: rl.Texture | None = None
    self._status = StatusFade(0.1)
    self._wheel_critical_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._wheel_angle_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._fm_readout_started_frame = -1
    self._fm_readout = None
    self._fm_readout_time = 0.0
    self._fm_lead_readout = LeadReadoutState()

    self._set_speed_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    self._set_speed_alpha = 0.0
    self._chestnut_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def set_wheel_critical_icon(self, critical: bool):
    """Set the wheel icon to critical or normal state."""
    self._show_wheel_critical = critical

  def set_torque_bar_hidden(self, hidden: bool):
    self._torque_bar.set_hidden(hidden)

  def set_can_draw_top_icons(self, can_draw_top_icons: bool):
    """Set whether to draw the top part of the HUD."""
    self._can_draw_top_icons = can_draw_top_icons

  def drawing_top_icons(self) -> bool:
    # whether we're drawing any top icons currently
    return bool(self._set_speed_alpha > 1e-2)

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if not ui_state.received_since_started("carState"):
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      return

    car_state = sm['carState']

    set_speed = car_state.vCruiseCluster
    engaged = sm['selfdriveState'].enabled
    if (set_speed != self.set_speed and engaged) or (engaged and not self._engaged):
      self._set_speed_changed_time = rl.get_time()
    if engaged != self._engaged:
      self._chestnut_fade_time = rl.get_time() if engaged else 0
    self._engaged = engaged
    self.set_speed = set_speed
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _has_fm_readout_data(self, *services: str) -> bool:
    sm = ui_state.sm
    return all(sm.alive[s] and sm.valid[s] and sm.recv_frame[s] >= ui_state.started_frame for s in services)

  def _draw_fm_top_readout(self, rect: rl.Rectangle) -> None:
    """FM Pilot: sent / limit now / Max Torque / EPAS check, one line at the top center (fm_torque_readout.py)."""
    sm = ui_state.sm
    mode = ui_state.fm_top_readout
    if mode == READOUT_OFF or not self._has_fm_readout_data('selfdriveState'):
      return
    if ui_state.started_frame != self._fm_readout_started_frame:
      self._fm_readout_started_frame = ui_state.started_frame
      self._fm_readout = None
      self._fm_lead_readout = LeadReadoutState()

    ss = sm['selfdriveState']
    turn_notice = (ss.alertSize == log.SelfdriveState.AlertSize.small and ss.alertStatus == log.SelfdriveState.AlertStatus.normal
                   and ss.alertType in TURN_NOTICE_TYPES)
    if ss.alertSize != log.SelfdriveState.AlertSize.none and not turn_notice:
      return
    if mode in (READOUT_CPU_AVG, READOUT_CPU_BUSIEST):
      self._draw_fm_cpu_readout(rect, mode, turn_notice)
      return
    if mode == READOUT_LEAD:
      self._draw_fm_lead_readout(rect, turn_notice)
      return
    if not self._has_fm_readout_data('carOutput', 'fmCarState', 'carControl'):
      self._fm_readout = None
      return
    now = rl.get_time()
    if self._fm_readout is None or now - self._fm_readout_time >= FM_READOUT_PERIOD:
      fm = sm['fmCarState']
      self._fm_readout = make_readout(sm['carOutput'].actuatorsOutput.torqueOutputCan, fm.steerCap,
                                      fm.maxTorque, fm.epasToiActive, sm['carControl'].latActive)
      self._fm_readout_time = now
    r = self._fm_readout
    if r is None:
      return

    font_size = FONT_SIZES.max_speed
    color = FM_READOUT_COLORS[r.state]
    text = f"{r.sent} / {r.cap} / {r.max_torque}"
    size = measure_text_cached(self._font_semi_bold, text, font_size)
    arrow_w, gap, mark_w = 16, 10, 24
    total = arrow_w + gap + size.x + gap + mark_w
    x = rect.x + rect.width / 2 - total / 2
    y = rect.y + 16
    if turn_notice:
      y += FM_TURN_NOTICE_OFFSET
    cy = y + size.y / 2

    # backing so it reads over a bright road
    rl.draw_rectangle_rounded(rl.Rectangle(x - 12, y - 4, total + 24, size.y + 8), 0.4, 8, rl.Color(0, 0, 0, 110))

    if r.direction != 0:
      half = arrow_w / 2
      ax = x + half
      if r.direction < 0:  # left
        rl.draw_triangle(rl.Vector2(ax - half, cy), rl.Vector2(ax + half, cy + half), rl.Vector2(ax + half, cy - half), color)
      else:
        rl.draw_triangle(rl.Vector2(ax + half, cy), rl.Vector2(ax - half, cy - half), rl.Vector2(ax - half, cy + half), color)

    rl.draw_text_ex(self._font_semi_bold, text, rl.Vector2(x + arrow_w + gap, y), font_size, 0, color)

    mx = x + arrow_w + gap + size.x + gap
    if r.accepted is True:
      rl.draw_line_ex(rl.Vector2(mx + 2, cy), rl.Vector2(mx + 9, cy + 8), 4, FM_CHECK_COLOR)
      rl.draw_line_ex(rl.Vector2(mx + 9, cy + 8), rl.Vector2(mx + 22, cy - 9), 4, FM_CHECK_COLOR)
    elif r.accepted is False:
      red = FM_READOUT_COLORS['refused']
      rl.draw_line_ex(rl.Vector2(mx + 3, cy - 9), rl.Vector2(mx + 21, cy + 9), 4, red)
      rl.draw_line_ex(rl.Vector2(mx + 3, cy + 9), rl.Vector2(mx + 21, cy - 9), 4, red)

  def _draw_fm_cpu_readout(self, rect: rl.Rectangle, mode: int, turn_notice: bool) -> None:
    """FM Pilot: core 4 / other cores / temperature, same place and style as the torque readout (fm_torque_readout.py)."""
    sm = ui_state.sm
    if not self._has_fm_readout_data('deviceState'):
      return
    ds = sm['deviceState']
    out = make_cpu_readout(mode, ds.cpuUsagePercent, ds.cpuTempC, ds.maxTempC)
    if out is None:
      return
    text, busy = out
    font_size = FONT_SIZES.max_speed
    color = FM_READOUT_COLORS['at_cap' if busy else 'normal']
    size = measure_text_cached(self._font_semi_bold, text, font_size)
    x = rect.x + rect.width / 2 - size.x / 2
    y = rect.y + 16 + (FM_TURN_NOTICE_OFFSET if turn_notice else 0)
    rl.draw_rectangle_rounded(rl.Rectangle(x - 12, y - 4, size.x + 24, size.y + 8), 0.4, 8, rl.Color(0, 0, 0, 110))
    rl.draw_text_ex(self._font_semi_bold, text, rl.Vector2(x, y), font_size, 0, color)

  def _draw_fm_lead_readout(self, rect: rl.Rectangle, turn_notice: bool) -> None:
    """FM Pilot: lead confidence / source / early radar matches / ghost drops, same place and style as the torque
    readout (fm_torque_readout.py). Drawn engaged or not: radard runs either way."""
    sm = ui_state.sm
    if not self._has_fm_readout_data('fmRadarLead'):
      self._fm_lead_readout = LeadReadoutState()
      return
    lead = sm['fmRadarLead']
    r = self._fm_lead_readout.update(rl.get_time(), float(lead.liveLeadProb), int(lead.source.raw), bool(lead.lowSpeedLead),
                                     bool(lead.ghostDropped), int(lead.ghostDrops), int(lead.earlyRadarMatches))
    font_size = FONT_SIZES.max_speed
    parts = [(r.prob, FM_LEAD_COLORS[r.prob_color]), (r.letter, FM_LEAD_COLORS[r.letter_color]),
             (r.counts, FM_LEAD_COLORS['white'])]
    space = measure_text_cached(self._font_semi_bold, " ", font_size).x
    sizes = [measure_text_cached(self._font_semi_bold, t, font_size) for t, _ in parts]
    total = sum(sz.x for sz in sizes) + space * (len(parts) - 1)
    height = max(sz.y for sz in sizes)
    x = rect.x + rect.width / 2 - total / 2
    y = rect.y + 16 + (FM_TURN_NOTICE_OFFSET if turn_notice else 0)
    rl.draw_rectangle_rounded(rl.Rectangle(x - 12, y - 4, total + 24, height + 8), 0.4, 8, rl.Color(0, 0, 0, 110))
    for (text, color), sz in zip(parts, sizes, strict=True):
      rl.draw_text_ex(self._font_semi_bold, text, rl.Vector2(x, y), font_size, 0, color)
      x += sz.x + space

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""

    self._torque_bar.render(rect)

    if self.is_cruise_set:
      self._draw_set_speed(rect)

    self._draw_fm_top_readout(rect)

    self._draw_model_source(rect)

  def render_steering_wheel(self, rect: rl.Rectangle) -> None:
    """Draw the steering wheel centered at the bottom of rect, after render() has updated state."""
    self._draw_steering_wheel(rect)

  def _draw_model_source(self, rect: rl.Rectangle) -> None:
    loading = ui_state.chestnut_state == ChestnutState.LOADING
    if loading:
      icon = self._txt_chestnut
      opacity = 0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0))
    elif ui_state.chestnut_state in (ChestnutState.UNCOMPILED, ChestnutState.FAILED):
      icon = self._txt_chestnut_orange
      opacity = 1.0
    elif ui_state.chestnut_state in (ChestnutState.ACTIVE, ChestnutState.WAITING):
      icon = self._txt_chestnut_green
      opacity = fm_jetlink_icon.DIMMED if ui_state.chestnut_state == ChestnutState.WAITING else 1.0
    else:
      return

    if icon is not self._chestnut_icon:
      self._chestnut_fade_time = rl.get_time()
      self._chestnut_icon = icon
    stays = fm_jetlink_icon.stays_on(ui_state.chestnut_state.value, ui_state.jetlink_view is not None)
    visible = loading or stays or rl.get_time() - self._chestnut_fade_time < SET_SPEED_PERSISTENCE
    alpha = self._chestnut_alpha_filter.update(visible)
    if alpha < 1e-2:
      return

    pos = rl.Vector2(rect.x + 14, rect.y + rect.height - 10 - icon.height)
    rl.draw_texture_ex(icon, pos, 0.0, 1.0, rl.Color(255, 255, 255, int(255 * opacity * alpha)))

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    engaged = self._status.weight(UIStatus.ENGAGED) + self._status.weight(UIStatus.LAT_ONLY)
    critical = self._wheel_critical_filter.update(self._show_wheel_critical) * engaged
    status_color = self._status.color({UIStatus.ENGAGED: WHEEL_GREEN, UIStatus.OVERRIDE: WHEEL_WHITE, UIStatus.DISENGAGED: WHEEL_WHITE})
    wheel_color = lerp_color(status_color, WHEEL_ORANGE, critical)
    wheel_txt = self._txt_wheel

    # pos
    pos_x = int(rect.x + rect.width / 2)
    pos_y = int(rect.y + rect.height - 10 - wheel_txt.height / 2)
    # ease between the live angle and rest on ignition changes; carState keeps its last angle once stopped
    rotation = -ui_state.sm['carState'].steeringAngleDeg * self._wheel_angle_filter.update(ui_state.started and ui_state.received_since_started('carState'))

    alpha = 255 * (1 - (1 - DIMMED_ICON_ALPHA) * self._status.weight(UIStatus.DISENGAGED))
    rl.draw_texture_pro(wheel_txt, rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height),
                        rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height),
                        rl.Vector2(wheel_txt.width / 2, wheel_txt.height / 2), rotation, rl.Color(wheel_color.r, wheel_color.g, wheel_color.b, int(alpha)))
    rl.draw_texture_pro(self._txt_wheel_highlight, rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height),
                        rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height),
                        rl.Vector2(wheel_txt.width / 2, wheel_txt.height / 2), rotation, rl.Color(255, 255, 255, int(alpha * engaged)))

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    alpha = self._set_speed_alpha_filter.update(0 < rl.get_time() - self._set_speed_changed_time < SET_SPEED_PERSISTENCE and self._can_draw_top_icons)
    alpha = self._set_speed_alpha = alpha * (1 - self._status.weight(UIStatus.DISENGAGED))
    if alpha < 1e-2:
      return

    x = rect.x
    y = rect.y

    # draw drop shadow
    circle_radius = 162 // 2
    rl.draw_circle_gradient(rl.Vector2(x + circle_radius, y + circle_radius), circle_radius,
                            rl.Color(0, 0, 0, int(255 / 2 * alpha)), rl.BLANK)

    set_speed_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))
    max_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))

    set_speed = self.set_speed
    if self.is_cruise_set and not ui_state.is_metric:
      set_speed *= KM_TO_MILE

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(set_speed))
    rl.draw_text_ex(
      self._font_display,
      set_speed_text,
      rl.Vector2(x + 13 + 4, y + 3 - 8 - 3 + 4),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

    max_text = tr("MAX")
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + 25, y + FONT_SIZES.set_speed - 7 + 4),
      FONT_SIZES.max_speed,
      0,
      max_color,
    )

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)
