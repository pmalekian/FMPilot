"""
FM Pilot Radar Badge: a small radar badge on the lead car in the comma 4 road view, showing what radard did with the lead
(fmRadarLead). Hidden while radar and camera agree (the normal case), so it only appears when something is different:

  amber, pulsing, ~2 s   Ignore Stationary Radar Ghosts just dropped a standing-still radar point; the car is followed
                         from the camera
  gray                   camera-only lead (no radar match, normal at a distance or when the radar loses the car)
  gray ring (hollow)     an early lead: the camera is only Lead Detection Sensitivity..50% sure of it, so openpilot's
                         fixed 50% wouldn't have it yet

A new state other than the ghost waits 0.3 s before it shows, and a shown state stays at least 1 s, so the badge doesn't
flicker with radard's 20 Hz. Display only; off by default (FM_RadarBadge).
"""
import math

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus

HIDDEN, GHOST, CAMERA, EARLY = 0, 1, 2, 3

GHOST_SHOW_S = 2.0   # how long a ghost drop keeps the badge amber
SETTLE_S = 0.3       # a new state (other than a ghost) must last this long before it shows
MIN_HOLD_S = 1.0     # a shown state stays at least this long

SOURCE_CAMERA = 2    # custom.FmRadarLead.Source.camera

GHOST_FILL = rl.Color(239, 159, 39, 255)
GHOST_GLYPH = rl.Color(65, 36, 2, 255)
CAMERA_FILL = rl.Color(136, 135, 128, 220)
CAMERA_GLYPH = rl.Color(44, 44, 42, 255)
EARLY_RING = rl.Color(180, 178, 169, 230)
PULSE_PERIOD_S = 0.8


def wanted_state(now: float, ghost_until: float, source: int, early: bool) -> int:
  if now < ghost_until:
    return GHOST
  if early:
    return EARLY
  if source == SOURCE_CAMERA:
    return CAMERA
  return HIDDEN


class RadarBadgeState:
  """Which badge to show, from fmRadarLead frame by frame. Pure logic, no drawing (tested on its own)."""

  def __init__(self):
    self.shown = HIDDEN
    self.shown_since = -math.inf
    self.ghost_until = -math.inf
    self._pending = HIDDEN
    self._pending_since = -math.inf
    self._ghost_drops = None

  def reset(self):
    self.__init__()

  def update(self, now: float, source: int, early: bool, ghost_dropped: bool, ghost_drops: int) -> int:
    # a drop seen directly, or one the UI missed between frames (the drive's count went up)
    new_drop = self._ghost_drops is not None and ghost_drops > self._ghost_drops
    self._ghost_drops = ghost_drops
    if ghost_dropped or new_drop:
      self.ghost_until = now + GHOST_SHOW_S

    want = wanted_state(now, self.ghost_until, source, early)
    if want != self._pending:
      self._pending, self._pending_since = want, now

    if want != self.shown:
      settled = now - self._pending_since >= SETTLE_S
      held = now - self.shown_since >= MIN_HOLD_S
      if want == GHOST or (settled and held):
        self.shown, self.shown_since = want, now
    return self.shown


def badge_center(chevron, sz_min: float = 15.0) -> tuple[float, float, float] | None:
  """Where the lead car is on screen, from the renderer's chevron (tip, then the two base corners), and the badge
  radius. The comma 4 doesn't draw the chevron itself; it only uses it for the position."""
  if not chevron or len(chevron) != 3:
    return None
  (rx, ry), (tx, ty), _ = chevron
  sz = max(ry - ty, sz_min)
  return tx, ty + sz * 0.35, min(max(sz * 0.6, 9.0), 18.0)


class FmRadarBadge:
  def __init__(self):
    self.state = RadarBadgeState()

  def update(self) -> int:
    sm = ui_state.sm
    now = rl.get_time()
    if not ui_state.fm_radar_badge or not sm.seen['fmRadarLead'] or sm.recv_frame['fmRadarLead'] < ui_state.started_frame:
      self.state.reset()
      return HIDDEN
    lead = sm['fmRadarLead']
    return self.state.update(now, int(lead.source.raw), bool(lead.earlyLead), bool(lead.ghostDropped), int(lead.ghostDrops))

  def draw(self, rect: rl.Rectangle, chevron) -> None:
    shown = self.update()
    if shown == HIDDEN or ui_state.status == UIStatus.DISENGAGED:
      return
    pos = badge_center(chevron)
    if pos is None:
      return
    x, y, r = pos[0] + rect.x, pos[1] + rect.y, pos[2]
    center = rl.Vector2(x, y)
    if shown == GHOST:
      pulse = 1.0 + 0.15 * math.sin(2 * math.pi * rl.get_time() / PULSE_PERIOD_S)
      rl.draw_circle_v(center, r * pulse, GHOST_FILL)
      _draw_glyph(center, r, GHOST_GLYPH)
    elif shown == CAMERA:
      rl.draw_circle_v(center, r, CAMERA_FILL)
      _draw_glyph(center, r, CAMERA_GLYPH)
    else:  # EARLY: hollow
      rl.draw_ring(center, r - max(2.0, r * 0.18), r, 0, 360, 36, EARLY_RING)
      _draw_glyph(center, r, EARLY_RING)


def _draw_glyph(center: rl.Vector2, r: float, color: rl.Color) -> None:
  """A radar mark: a dot low in the badge and two arcs opening upward (screen y points down, so 270 deg is up)."""
  base = rl.Vector2(center.x, center.y + r * 0.35)
  w = max(1.5, r * 0.14)
  rl.draw_circle_v(base, max(1.5, r * 0.13), color)
  for k in (0.42, 0.75):
    rl.draw_ring(base, r * k - w / 2, r * k + w / 2, 225, 315, 12, color)
