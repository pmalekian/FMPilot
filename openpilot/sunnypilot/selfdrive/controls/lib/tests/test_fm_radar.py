"""FM Pilot radar lead settings in radard: Stationary Radar Ghost Guard and Lead Detection Sensitivity."""
from types import SimpleNamespace

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.radard import KalmanParams, Track, get_lead
from openpilot.sunnypilot.selfdrive.controls.lib.fm_radar import (GHOST_VISION_BRAKING, GHOST_VISION_MIN_SPEED, PARAM_GUARD,
                                                                  PARAM_THRESHOLD, FmRadarSettings, stationary_ghost,
                                                                  threshold_from_percent)

CP = SimpleNamespace(brand='rivian', flags=0)
CP_SP = SimpleNamespace(flags=0)
MPH = 0.44704


def lead_msg(x=12.0, y=3.6, v=8.5, a=0.0, prob=0.99):
  """Camera lead in the model frame (y right-positive; radard compares the radar's yRel to -y)."""
  return SimpleNamespace(x=[x], xStd=[2.0], y=[y], yStd=[1.0], v=[v], vStd=[1.5], a=[a], aStd=[1.0], prob=prob)


def track(ident, d_rel, y_rel, v_lead, v_ego):
  t = Track(ident, v_lead, KalmanParams(DT_MDL))
  t.update(d_rel, y_rel, v_lead - v_ego, v_lead)
  return t


def lead(tracks, msg, v_ego, guard, low_speed_override=True):
  return get_lead(v_ego, True, {t.identifier: t for t in tracks}, msg, v_ego, msg.prob, CP, CP_SP,
                  low_speed_override=low_speed_override, stationary_guard=guard)


# --- the rule itself ---

def test_ghost_rule():
  assert stationary_ghost(0.0, 8.5, 0.0)            # barrier beside a car doing 19 mph (00000038--19)
  assert stationary_ghost(-0.4, 6.0, -1.0)          # small negative speed, gentle braking
  assert not stationary_ghost(1.5, 8.5, 0.0)        # radar point moving: not a ghost
  assert not stationary_ghost(0.0, 3.2, 0.0)        # camera's lead nearly stopped (real car at a light)
  assert not stationary_ghost(0.0, GHOST_VISION_MIN_SPEED, 0.0)  # at the line itself: keep the radar
  assert not stationary_ghost(0.0, 8.5, -2.0)       # camera sees the lead braking hard: keep the radar
  assert not stationary_ghost(0.0, 8.5, GHOST_VISION_BRAKING)


# --- the ramp phantom: 00000038--19 +18.0 s ---

def ramp_phantom():
  v_ego = 15.3 * MPH
  barrier = track(35184, 16.0, 0.8, 0.0, v_ego)  # standing still, 16 m ahead, 0.8 m to the left
  msg = lead_msg(x=12.7, y=4.1, v=18.4 * MPH)     # the real car, camera only, 4.1 m to the right on the curve
  return [barrier], msg, v_ego


def test_ramp_phantom_is_paired_today():
  tracks, msg, v_ego = ramp_phantom()
  out = lead(tracks, msg, v_ego, guard=False)
  assert out['radar'] and out['vLead'] == pytest.approx(0.0)


def test_ramp_phantom_follows_the_camera_with_the_guard():
  tracks, msg, v_ego = ramp_phantom()
  out = lead(tracks, msg, v_ego, guard=True)
  assert out['present'] and not out['radar']
  assert out['vLead'] == pytest.approx(18.4 * MPH, abs=0.5)
  assert out['dRel'] == pytest.approx(12.7 - 1.52)


# --- cases the guard must leave alone ---

def test_real_car_matched_by_radar_is_unchanged():
  v_ego = 19 * MPH
  car = track(1, 10.5, -3.5, 18.5 * MPH, v_ego)
  msg = lead_msg(x=12.0, y=3.5, v=18.5 * MPH)
  for guard in (False, True):
    out = lead([car], msg, v_ego, guard)
    assert out['radar'] and out['radarTrackId'] == 1


def test_car_pulling_away_from_a_light_is_unchanged():
  # 00000039--48 +23 s: radar ~2 mph at 50 m, camera 2-9 mph
  v_ego = 24 * MPH
  car = track(108114, 49.6, 0.0, 0.5, v_ego)
  for cam_mph in (3.4, 8.9):
    msg = lead_msg(x=51.0, y=0.0, v=cam_mph * MPH)
    for guard in (False, True):
      out = lead([car], msg, v_ego, guard)
      assert out['radar'] and out['radarTrackId'] == 108114, (cam_mph, guard)


def test_car_stopping_hard_keeps_the_radar():
  # the camera's speed still lags at 9 m/s, but it sees the lead braking hard
  v_ego = 12.0
  car = track(7, 20.0, 0.0, 0.3, v_ego)
  msg = lead_msg(x=21.5, y=0.0, v=9.0, a=-3.0)
  out = lead([car], msg, v_ego, guard=True)
  assert out['radar'] and out['radarTrackId'] == 7


def test_low_speed_path_still_stops_for_things_ahead():
  # below 4 m/s ego speed a radar point straight ahead is a lead without the camera, guard or not
  v_ego = 3.0
  obstacle = track(9, 8.0, 0.2, 0.0, v_ego)
  msg = lead_msg(x=30.0, y=0.0, v=8.0, prob=0.99)
  out = lead([obstacle], msg, v_ego, guard=True)
  assert out['radar'] and out['radarTrackId'] == 9


def test_lead_two_with_the_guard():
  tracks, msg, v_ego = ramp_phantom()
  out = lead(tracks, msg, v_ego, guard=True, low_speed_override=False)
  assert out['present'] and not out['radar']


# --- settings ---

class FakeParams:
  def __init__(self, guard):
    self.values = {PARAM_GUARD: guard}

  def get_bool(self, key):
    return bool(self.values.get(key))


def test_settings_default_off_and_reread_once_per_second():
  assert FmRadarSettings().guard is False
  params = FakeParams(False)
  s = FmRadarSettings(params)
  assert s.guard is False
  params.values[PARAM_GUARD] = True
  for _ in range(int(1.0 / DT_MDL) + 1):
    s.update()
  assert s.guard is True


def test_settings_read_failure_is_off():
  class Broken:
    def get_bool(self, key):
      raise RuntimeError
  assert FmRadarSettings(Broken()).guard is False


# --- Lead Detection Sensitivity (from FrogPilot) ---

def far_car(prob):
  v_ego = 20.0
  car = track(3, 75.0, 0.0, 12.0, v_ego)
  return [car], lead_msg(x=76.5, y=0.0, v=12.0, prob=prob), v_ego


def lead_at(tracks, msg, v_ego, threshold):
  return get_lead(v_ego, True, {t.identifier: t for t in tracks}, msg, v_ego, msg.prob, CP, CP_SP,
                  low_speed_override=True, lead_threshold=threshold)


def test_default_threshold_is_openpilots():
  tracks, msg, v_ego = far_car(prob=0.45)
  assert not lead_at(tracks, msg, v_ego, 0.5)['present']
  assert not get_lead(v_ego, True, {3: tracks[0]}, msg, v_ego, msg.prob, CP, CP_SP)['present']  # default argument


@pytest.mark.parametrize("threshold,prob,present", [(0.5, 0.45, False), (0.4, 0.45, True), (0.35, 0.38, True),
                                                    (0.35, 0.30, False), (0.25, 0.30, True)])
def test_lower_threshold_picks_up_the_lead_sooner(threshold, prob, present):
  tracks, msg, v_ego = far_car(prob)
  out = lead_at(tracks, msg, v_ego, threshold)
  assert out['present'] == present
  if present:
    assert out['radar'] and out['radarTrackId'] == 3  # paired with the radar, not just the camera


def test_threshold_also_gates_the_camera_only_lead():
  msg = lead_msg(x=60.0, y=0.0, v=10.0, prob=0.42)
  assert not lead_at([], msg, 20.0, 0.5)['present']
  out = lead_at([], msg, 20.0, 0.4)
  assert out['present'] and not out['radar']


@pytest.mark.parametrize("value,expected", [(50, 0.5), (40, 0.4), (35, 0.35), (25, 0.25), (10, 0.25), (80, 0.5),
                                            (None, 0.5), ("x", 0.5), (float('nan'), 0.5)])
def test_threshold_setting_is_clamped(value, expected):
  assert threshold_from_percent(value) == pytest.approx(expected)


def test_settings_read_threshold():
  class P:
    values = {PARAM_GUARD: False, PARAM_THRESHOLD: 35}

    def get_bool(self, key):
      return bool(self.values[key])

    def get(self, key, return_default=False):
      return self.values[key]
  assert FmRadarSettings(P()).threshold == pytest.approx(0.35)
  assert FmRadarSettings().threshold == pytest.approx(0.5)
