"""FM Pilot Radar Badge: which badge shows, from fmRadarLead frame by frame (hidden while radar and camera agree)."""
from openpilot.selfdrive.ui.sunnypilot.onroad.fm_radar_badge import (CAMERA, EARLY, GHOST, GHOST_SHOW_S, HIDDEN, MIN_HOLD_S,
                                                                     SETTLE_S, RadarBadgeState, badge_center)

NONE, RADAR, CAM = 0, 1, 2
DT = 0.05  # radard's 20 Hz


def run(state, t0, t1, source, early=False, ghost=False, drops=0):
  t, shown = t0, []
  while t < t1 - 1e-9:
    shown.append(state.update(t, source, early, ghost, drops))
    t += DT
  return shown


def test_hidden_while_radar_and_camera_agree():
  s = RadarBadgeState()
  assert set(run(s, 0, 5, RADAR)) == {HIDDEN}
  assert set(run(RadarBadgeState(), 0, 5, NONE)) == {HIDDEN}


def test_ghost_shows_at_once_for_two_seconds():
  s = RadarBadgeState()
  run(s, 0, 1, RADAR)
  assert s.update(1.0, CAM, False, True, 1) == GHOST       # no settle wait for a ghost
  run(s, 1.05, 1.5, CAM, ghost=True, drops=1)               # the drop lasts 0.5 s
  shown = run(s, 1.5, 1.0 + 1.5 + GHOST_SHOW_S + 1, RADAR, drops=1)
  t_end = 1.5 + shown.index(HIDDEN) * DT
  assert 1.45 + GHOST_SHOW_S <= t_end <= 1.45 + GHOST_SHOW_S + SETTLE_S + 2 * DT  # 2 s after the last drop, then settle


def test_missed_drop_frame_still_shows_the_ghost():
  s = RadarBadgeState()
  s.update(0.0, CAM, False, False, 3)
  assert s.update(0.05, CAM, False, False, 4) == GHOST  # the UI missed the frame but the drive's count went up


def test_camera_only_waits_and_holds():
  s = RadarBadgeState()
  run(s, 0, 2, RADAR)
  assert run(s, 2, 2 + SETTLE_S - DT, CAM)[-1] == HIDDEN    # a short camera-only blip doesn't show
  s = RadarBadgeState()
  run(s, 0, 2, RADAR)
  assert run(s, 2, 2 + SETTLE_S + 2 * DT, CAM)[-1] == CAMERA  # lasting it does
  after = run(s, 2 + SETTLE_S + 2 * DT, 2 + SETTLE_S + MIN_HOLD_S, RADAR)
  assert set(after) == {CAMERA}                             # and stays at least 1 s
  assert run(s, 4, 5, RADAR)[-1] == HIDDEN


def test_early_lead_is_hollow_and_ghost_wins():
  s = RadarBadgeState()
  assert run(s, 0, 1, RADAR, early=True)[-1] == EARLY
  assert s.update(1.0, CAM, True, True, 1) == GHOST


def test_reset():
  s = RadarBadgeState()
  s.update(0.0, CAM, False, True, 1)
  s.reset()
  assert s.shown == HIDDEN and s.update(0.05, RADAR, False, False, 1) == HIDDEN


def test_badge_position():
  assert badge_center([]) is None
  x, y, r = badge_center([(130.0, 120.0), (100.0, 100.0), (70.0, 120.0)])
  assert x == 100.0 and y == 107.0 and 9.0 <= r <= 18.0
