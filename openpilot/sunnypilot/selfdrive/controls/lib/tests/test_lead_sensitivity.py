"""
Tests for Custom Lead Sensitivity in radard.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import random
from types import SimpleNamespace

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.radard import KalmanParams, Track, get_lead, get_lead_custom
from openpilot.sunnypilot.selfdrive.controls.lib.lead_sensitivity import (
  PARAM_CAMERA, PARAM_ENABLED, PARAM_RADAR, LeadSensitivityConfig, LeadSensitivitySettings, confidence_from_percent,
)

CP = SimpleNamespace(brand='rivian', flags=0)
CP_SP = SimpleNamespace(flags=0)
STOCK = LeadSensitivityConfig()
RECOMMENDED = LeadSensitivityConfig(camera=0.45, radar=0.35)


def lead_msg(x=40.0, y=0.0, v=20.0, a=0.0, prob=0.99):
  """Camera lead in the model frame (y right-positive; radard compares the radar's yRel to -y)."""
  return SimpleNamespace(x=[x], xStd=[2.0], y=[y], yStd=[1.0], v=[v], vStd=[1.5], a=[a], aStd=[1.0], prob=prob)


def track(ident, d_rel, y_rel, v_lead, v_ego):
  t = Track(ident, v_lead, KalmanParams(DT_MDL))
  t.update(d_rel, y_rel, v_lead - v_ego, v_lead)
  return t


def custom(tracks, msg, v_ego, config, prob=None, low_speed_override=True, **kwargs):
  prob = msg.prob if prob is None else prob
  return get_lead_custom(v_ego, True, {t.identifier: t for t in tracks}, msg, v_ego, prob, CP, CP_SP, config,
                         low_speed_override=low_speed_override, **kwargs)


def stock(tracks, msg, v_ego, prob=None, low_speed_override=True):
  prob = msg.prob if prob is None else prob
  return get_lead(v_ego, True, {t.identifier: t for t in tracks}, msg, v_ego, prob, CP, CP_SP,
                  low_speed_override=low_speed_override)


# --- settings ---

def test_confidence_from_percent():
  assert confidence_from_percent(50) == 0.5
  assert confidence_from_percent(35) == pytest.approx(0.35)
  assert confidence_from_percent(10) == 0.25
  assert confidence_from_percent(90) == 0.5
  for bad in (None, "x", float('nan'), float('inf')):
    assert confidence_from_percent(bad) == 0.5


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get(self, key):
    return self.values.get(key)


def test_settings_off_by_default():
  assert LeadSensitivitySettings().config is None
  assert LeadSensitivitySettings(FakeParams()).config is None


def test_sub_settings_ignored_while_off():
  params = FakeParams(**{PARAM_ENABLED: False, PARAM_CAMERA: 45, PARAM_RADAR: 35})
  assert LeadSensitivitySettings(params).config is None


def test_settings_on():
  params = FakeParams(**{PARAM_ENABLED: True, PARAM_CAMERA: 45, PARAM_RADAR: 35})
  config = LeadSensitivitySettings(params).config
  assert config.camera == pytest.approx(0.45) and config.radar == pytest.approx(0.35)
  assert config.match == pytest.approx(0.35)


def test_settings_on_with_defaults_is_stock():
  assert LeadSensitivitySettings(FakeParams(**{PARAM_ENABLED: True})).config == STOCK


def test_settings_reread_once_per_second():
  params = FakeParams(**{PARAM_ENABLED: True})
  settings = LeadSensitivitySettings(params)
  settings.update()  # frame 0 reads
  params.values[PARAM_ENABLED] = False
  for _ in range(round(1 / DT_MDL) - 1):
    settings.update()
  assert settings.config is not None
  settings.update()
  assert settings.config is None


# --- radard ---

def test_stock_cutoffs_match_stock_get_lead():
  rng = random.Random(0)
  for _ in range(2000):
    v_ego = rng.uniform(0., 35.)
    tracks = [track(i, rng.uniform(1., 120.), rng.uniform(-4., 4.), rng.uniform(0., 35.), v_ego) for i in range(rng.randint(0, 4))]
    msg = lead_msg(x=rng.uniform(2., 120.), y=rng.uniform(-4., 4.), v=rng.uniform(0., 35.), a=rng.uniform(-4., 2.),
                   prob=rng.uniform(0., 1.))
    low_speed_override = rng.random() < 0.5
    assert custom(tracks, msg, v_ego, STOCK, low_speed_override=low_speed_override) == \
           stock(tracks, msg, v_ego, low_speed_override=low_speed_override)


def test_radar_confirmed_car_followed_below_stock_cutoff():
  v_ego = 25.
  car = track(1, 58.5, 0., 22., v_ego)
  msg = lead_msg(x=60., v=22., prob=0.38)
  assert not stock([car], msg, v_ego)['present']
  out = custom([car], msg, v_ego, RECOMMENDED)
  assert out['present'] and out['radar'] and out['radarTrackId'] == 1


def test_camera_only_car_needs_camera_cutoff():
  v_ego = 25.
  msg = lead_msg(x=60., v=22.)
  assert not custom([], msg, v_ego, RECOMMENDED, prob=0.40)['present']
  out = custom([], msg, v_ego, RECOMMENDED, prob=0.47)
  assert out['present'] and not out['radar']


def test_radar_cutoff_above_camera_has_no_effect():
  v_ego = 25.
  car = track(1, 58.5, 0., 22., v_ego)
  msg = lead_msg(x=60., v=22., prob=0.42)
  out = custom([car], msg, v_ego, LeadSensitivityConfig(camera=0.40, radar=0.50))
  assert out['radar'] and out['radarTrackId'] == 1


def test_low_speed_override_unchanged():
  v_ego = 3.
  obstacle = track(9, 8., 0.2, 0., v_ego)
  msg = lead_msg(x=30., v=8., prob=0.99)
  out = custom([obstacle], msg, v_ego, RECOMMENDED)
  assert out['radar'] and out['radarTrackId'] == 9
