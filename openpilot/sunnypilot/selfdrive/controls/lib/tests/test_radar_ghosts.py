"""
Tests for Ignore Stationary Radar Ghosts in radard.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from openpilot.selfdrive.controls.radard import RadarD
from openpilot.sunnypilot.selfdrive.controls.lib.lead_sensitivity import (
  BLEND, DIRECT, GHOST_VISION_BRAKING, GHOST_VISION_MIN_SPEED, HOLD, PARAM_ENABLED, PARAM_GHOSTS, PARAM_SMOOTH, LeadHandoff,
  LeadSensitivityConfig,
  stationary_ghost,
)
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_lead_handoffs import EGO, FRAMES, PATH, measure, run
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_lead_sensitivity import (
  CP, CP_SP, FakeParams, custom, lead_msg, stock, track,
)

MPH = 0.44704
GUARD = LeadSensitivityConfig(ignore_ghosts=True)


def test_ghost_rule():
  assert stationary_ghost(0.0, 8.5, 0.0)  # barrier beside a car doing 19 mph
  assert stationary_ghost(-0.4, 6.0, -1.0)  # small negative speed, gentle braking
  assert not stationary_ghost(1.5, 8.5, 0.0)  # radar point moving: not a ghost
  assert not stationary_ghost(0.0, 3.2, 0.0)  # camera's car nearly stopped (real car at a light)
  assert not stationary_ghost(0.0, GHOST_VISION_MIN_SPEED, 0.0)
  assert not stationary_ghost(0.0, 8.5, -2.0)  # camera sees the car braking hard: keep the radar
  assert not stationary_ghost(0.0, 8.5, GHOST_VISION_BRAKING)


def ramp_ghost():
  """An on-ramp: the real car ahead on the curve isn't a radar point, a barrier 16 m ahead is."""
  v_ego = 15.3 * MPH
  barrier = track(1, 16.0, 0.8, 0.0, v_ego)
  msg = lead_msg(x=12.7, y=4.1, v=18.4 * MPH)
  return [barrier], msg, v_ego


def test_ramp_ghost_is_paired_by_stock():
  tracks, msg, v_ego = ramp_ghost()
  out = stock(tracks, msg, v_ego)
  assert out['radar'] and out['vLead'] == pytest.approx(0.0)


@pytest.mark.parametrize('low_speed_override', (True, False))
def test_ramp_ghost_follows_the_camera_with_the_guard(low_speed_override):
  tracks, msg, v_ego = ramp_ghost()
  out = custom(tracks, msg, v_ego, GUARD, low_speed_override=low_speed_override)
  assert out['present'] and not out['radar']
  assert out['vLead'] == pytest.approx(18.4 * MPH, abs=0.5)
  assert out['dRel'] == pytest.approx(12.7 - 1.52)


def test_ghost_fallback_still_needs_the_camera_cutoff():
  ghost = [track(7, 32.0, 0.0, 0.0, EGO)]
  config = LeadSensitivityConfig(camera=.50, radar=.30, ignore_ghosts=True)
  assert not custom(ghost, measure(speed=8.0, confidence=.45), EGO, config)['present']
  out = custom(ghost, measure(speed=8.0, confidence=.55), EGO, config)
  assert out['present'] and not out['radar']


def test_real_car_matched_by_radar_is_unchanged():
  v_ego = 19 * MPH
  car = track(1, 10.5, -3.5, 18.5 * MPH, v_ego)
  msg = lead_msg(x=12.0, y=3.5, v=18.5 * MPH)
  out = custom([car], msg, v_ego, GUARD)
  assert out['radar'] and out['radarTrackId'] == 1


def test_car_pulling_away_from_a_light_is_unchanged():
  # radar reads the car at ~1 mph 50 m ahead while the camera already reads 3-9 mph
  v_ego = 24 * MPH
  car = track(2, 49.6, 0.0, 0.5, v_ego)
  for cam_mph in (3.4, 8.9):
    out = custom([car], lead_msg(x=51.0, y=0.0, v=cam_mph * MPH), v_ego, GUARD)
    assert out['radar'] and out['radarTrackId'] == 2, cam_mph


def test_car_stopping_hard_keeps_the_radar():
  # the camera's speed still lags at 9 m/s, but it sees the car braking hard
  v_ego = 12.0
  car = track(3, 20.0, 0.0, 0.3, v_ego)
  out = custom([car], lead_msg(x=21.5, y=0.0, v=9.0, a=-3.0), v_ego, GUARD)
  assert out['radar'] and out['radarTrackId'] == 3


def test_low_speed_override_still_stops_for_things_ahead():
  v_ego = 3.0
  obstacle = track(9, 8.0, 0.2, 0.0, v_ego)
  out = custom([obstacle], lead_msg(x=30.0, y=0.0, v=8.0), v_ego, GUARD)
  assert out['radar'] and out['radarTrackId'] == 9


def test_ghost_ends_a_hold():
  handoff = LeadHandoff()
  config = LeadSensitivityConfig(camera=.50, radar=.30, smooth=True, ignore_ghosts=True)
  hold = {'handoff': handoff, 'model_path': PATH}
  custom([track(7, 32.0, 0.0, 8.0, EGO)], measure(speed=8.0), EGO, config, **hold)
  custom([], measure(speed=8.0, confidence=.40), EGO, config, **hold)
  assert handoff.phase == HOLD
  out = custom([track(8, 32.0, 0.0, 0.0, EGO)], measure(speed=8.0, confidence=.40), EGO, config, **hold)
  assert not out['present'] and handoff.estimate is None


@pytest.mark.parametrize('toggle_guard', (False, True))
def test_radard_toggling_the_guard_resets_handoffs(toggle_guard):
  params = FakeParams(**{PARAM_ENABLED: True, PARAM_SMOOTH: True})
  radard = RadarD(CP, CP_SP, params=params)
  run(radard, FRAMES[:2])
  assert radard.radar_state.leadOne.radar
  if toggle_guard:
    params.values[PARAM_GHOSTS] = True
    radard.lead_sensitivity.read()
  run(radard, FRAMES[4:5])  # the same car, now camera only
  assert radard.lead_handoffs[0].phase == (DIRECT if toggle_guard else BLEND)
