"""
Tests for Smooth Lead Handoffs in radard.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest

from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.selfdrive.controls.radard import RadarD, get_lead, get_lead_custom
from openpilot.sunnypilot.selfdrive.controls.lib.lead_sensitivity import (
  BLEND, BLEND_FRAMES, DIRECT, HOLD, HOLD_FRAMES, PARAM_CAMERA, PARAM_ENABLED, PARAM_RADAR, PARAM_SMOOTH, LeadHandoff,
  LeadSensitivityConfig,
)
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_lead_sensitivity import CP, CP_SP, FakeParams, lead_msg, track

EGO = 12.0
PATH = SimpleNamespace(x=[0.0, 100.0], y=[0.0, 0.0])


def measure(gap=32.0, speed=12.0, lateral=0.0, confidence=0.9, accel=0.0):
  return lead_msg(x=gap + 1.52, v=speed, y=-lateral, a=accel, prob=confidence)


def select(camera, tracks=(), *, radar=0.30, vision=0.50, handoff=None, slot=0, v_ego=EGO, probability=None,
           model_path=PATH):
  config = LeadSensitivityConfig(camera=vision, radar=radar, smooth=handoff is not None)
  return get_lead_custom(v_ego, True, {t.identifier: t for t in tracks}, camera, v_ego,
                         camera.prob if probability is None else probability, CP, CP_SP, config,
                         low_speed_override=slot == 0, handoff=handoff, model_path=model_path if handoff else None)


@pytest.mark.parametrize('slot', (0, 1))
@pytest.mark.parametrize(('radar', 'camera', 'p', 'has_track', 'source'), (
  (.30, .50, .30, True, 'none'),
  (.30, .50, .40, True, 'radar'),
  (.30, .50, .40, False, 'none'),
  (.30, .50, .50, False, 'none'),
  (.30, .50, .51, False, 'camera'),
  (.50, .40, .40, True, 'none'),
  (.50, .40, .45, True, 'radar'),
  (.50, .40, .45, False, 'camera'),
  (.50, .50, .50, True, 'none'),
  (.50, .50, .51, True, 'radar'),
))
@pytest.mark.parametrize('smooth', (False, True))
def test_cutoffs_with_and_without_handoffs(slot, radar, camera, p, has_track, source, smooth):
  points = [track(7, 32.0, 0.0, EGO, EGO)] if has_track else []
  lead = select(measure(confidence=p), points, radar=radar, vision=camera, slot=slot,
                handoff=LeadHandoff() if smooth else None)
  assert ('radar' if lead.get('radar') else 'camera' if lead['present'] else 'none') == source


@pytest.mark.parametrize('slot', (0, 1))
def test_first_frame_with_handoffs_matches_stock_at_stock_cutoffs(slot):
  candidate = track(7, 32.0, 0.0, 11.0, EGO)
  for confidence, points in ((.5, [candidate]), (.51, [candidate]), (.8, []), (.9, [candidate])):
    camera = measure(confidence=confidence, speed=8.0)
    handoff = LeadHandoff()
    out = select(camera, points, radar=.5, vision=.5, slot=slot, handoff=handoff)
    stock = get_lead(EGO, True, {point.identifier: point for point in points}, camera, EGO, confidence, CP, CP_SP,
                     low_speed_override=slot == 0)
    assert out == stock and handoff.phase == DIRECT


@pytest.mark.parametrize('slot', (0, 1))
def test_low_speed_override_keeps_raw_radar_lead_and_clears_handoff(slot):
  low_speed_point = track(7, 5.0, 0.0, 0.0, 3.0)
  handoff = LeadHandoff()
  select(measure(), [track(1, 32.0, 0.0, EGO, EGO)], handoff=handoff)
  lead = select(measure(confidence=.10), [low_speed_point], handoff=handoff, slot=slot, v_ego=3.0)
  assert lead['present'] == (slot == 0)
  assert handoff.phase == DIRECT and handoff.measurement is None


@pytest.mark.parametrize('radar_first', (True, False))
@pytest.mark.parametrize('change', (-1.0, 1.0))
def test_same_car_switch_is_asymmetric_for_both_sensors(radar_first, change):
  handoff = LeadHandoff()
  radar = track(7, 32.0 + (0 if radar_first else change), 0.0, EGO + (0 if radar_first else change), EGO)
  camera = measure(gap=32.0 + (change if radar_first else 0), speed=EGO + (change if radar_first else 0))
  before = select(camera, [radar] if radar_first else [], handoff=handoff)
  after = select(camera, [] if radar_first else [radar], handoff=handoff)
  assert handoff.phase == BLEND and after['radar'] != before['radar']
  expected = change if change < 0 else change / BLEND_FRAMES  # bad news at once, good news eased in
  assert after['dRel'] == pytest.approx(before['dRel'] + expected)
  assert after['vLead'] == pytest.approx(before['vLead'] + expected)
  assert after['vRel'] == pytest.approx(before['vRel'] + expected)


def test_blend_finishes_and_does_not_lag_later_updates():
  handoff = LeadHandoff()
  select(measure(), [track(7, 32.0, 0.0, EGO, EGO)], handoff=handoff)
  for _ in range(BLEND_FRAMES):
    lead = select(measure(34.0), handoff=handoff)
  assert lead['dRel'] == pytest.approx(34.0)
  lead = select(measure(35.0), handoff=handoff)
  assert lead['dRel'] == pytest.approx(35.0) and handoff.phase == DIRECT


def test_cut_in_resets_the_previous_cars_state():
  handoff = LeadHandoff()
  select(measure(), [track(7, 32.0, 0.0, EGO, EGO)], handoff=handoff)
  out = select(measure(gap=15.0, speed=6.0, confidence=.46), handoff=handoff)
  assert not out['present'] and handoff.estimate is None
  out = select(measure(gap=15.0, speed=6.0, confidence=.70, accel=-2.5), handoff=handoff)
  assert handoff.phase == DIRECT and out['dRel'] == 15.0 and out['aLeadK'] == -2.5


def test_lead_braking_during_blend_passes_through_on_the_first_frame():
  handoff = LeadHandoff()
  select(measure(), [track(7, 32.0, 0.0, EGO, EGO)], handoff=handoff)
  select(measure(34.0, 13.0), handoff=handoff)
  out = select(measure(31.5, 10.0, accel=-3.0), handoff=handoff)
  assert handoff.phase == BLEND
  assert (out['dRel'], out['vLead'], out['aLeadK']) == pytest.approx((31.5, 10.0, -3.0))


def test_hold_predicts_gap_and_times_out():
  handoff = LeadHandoff()
  select(measure(speed=10.0), [track(7, 32.0, 0.0, 10.0, EGO)], handoff=handoff)
  for i in range(1, HOLD_FRAMES + 1):
    lead = select(measure(32.0 - i * .1, speed=10.0, confidence=.40), handoff=handoff)
    assert handoff.phase == HOLD and lead['dRel'] == pytest.approx(32.0 - i * .1)
    assert lead['radarTrackId'] == 7
  lead = select(measure(31.0, speed=10.0, confidence=.40), handoff=handoff)
  assert not lead['present'] and handoff.phase == DIRECT


@pytest.mark.parametrize('end', ('low confidence', 'leaving path'))
def test_hold_releases_immediately(end):
  handoff = LeadHandoff()
  select(measure(speed=8.0), [track(7, 32.0, 0.0, 8.0, EGO)], handoff=handoff)
  select(measure(speed=8.0, confidence=.40), handoff=handoff)
  assert handoff.phase == HOLD
  wrong_path = SimpleNamespace(x=[0.0, 100.0], y=[3.0, 3.0])
  camera = measure(speed=8.0, confidence=.01 if end == 'low confidence' else .40)
  lead = select(camera, handoff=handoff, model_path=wrong_path if end == 'leaving path' else PATH)
  assert not lead['present'] and handoff.phase == DIRECT and handoff.estimate is None


def test_camera_clearing_its_cutoff_blends_from_hold():
  handoff = LeadHandoff()
  select(measure(), [track(7, 32.0, 0.0, EGO, EGO)], handoff=handoff)
  select(measure(confidence=.40), handoff=handoff)
  assert handoff.phase == HOLD
  lead = select(measure(33.0, confidence=.55), handoff=handoff)
  assert handoff.phase == BLEND and not lead['radar']
  assert lead['dRel'] == pytest.approx(32.0 + 1 / BLEND_FRAMES)


@pytest.mark.parametrize('radar', (False, True))
def test_release_margin_is_strict_and_same_car_only(radar):
  handoff = LeadHandoff()
  points = [track(7, 32.0, 0.0, EGO, EGO)] if radar else []
  options = {'radar': .50, 'vision': .50, 'handoff': handoff}
  assert select(measure(confidence=.51), points, **options)['present']
  assert select(measure(confidence=.46), points, **options)['present']
  assert not select(measure(confidence=.45), points, **options)['present']
  assert not select(measure(confidence=.46), points, **options)['present']  # released: needs the full cutoff again


def test_followed_radar_point_yields_when_it_no_longer_fits_the_car():
  handoff = LeadHandoff()
  original = track(7, 32.0, 0.0, EGO, EGO)
  alternate = track(8, 33.0, 0.0, EGO, EGO)
  select(measure(), [original], handoff=handoff)
  assert select(measure(33.0), [original, alternate], handoff=handoff)['radarTrackId'] == 7
  moved_off_path = track(7, 32.0, 3.0, EGO, EGO)
  assert select(measure(33.0), [moved_off_path, alternate], handoff=handoff)['radarTrackId'] == 8


@pytest.mark.parametrize(('far', 'near'), ((40.0, 30.0), (95.0, 80.0), (120.0, 100.0)))
def test_far_followed_point_yields_to_a_closer_cut_in(far, near):
  # radard's pairing allows 25% of the gap, so at long range the followed point used to pass while the camera
  # had already moved to a car 15-20 m closer
  v_ego, path = 30.0, SimpleNamespace(x=[0.0, 200.0], y=[0.0, 0.0])
  handoff = LeadHandoff()
  followed = track(1, far, 0.0, 28.0, v_ego)
  for _ in range(5):
    select(measure(far, 28.0), [followed], handoff=handoff, v_ego=v_ego, model_path=path)
  cut_in = track(2, near, 0.2, 27.0, v_ego)
  camera = measure(near, 27.0, lateral=0.2)
  lead = select(camera, [followed, cut_in], handoff=handoff, v_ego=v_ego, model_path=path)
  assert lead['radarTrackId'] == 2 and lead['dRel'] == near
  stock = get_lead(v_ego, True, {1: followed, 2: cut_in}, camera, v_ego, camera.prob, CP, CP_SP)
  assert stock['radarTrackId'] == 2


# --- radard ---

class RadarInputs(dict):
  def __init__(self, leads):
    super().__init__()
    self.logMonoTime = {'modelV2': 0}
    self.recv_frame = {'carState': 0}
    self.seen = {'modelV2': True}
    self['carState'] = SimpleNamespace(vEgo=EGO)
    model = log.ModelDataV2.new_message()
    model.velocity.x = [EGO]
    model.position.x, model.position.y = PATH.x, PATH.y
    model.init('leadsV3', len(leads))
    for builder, lead in zip(model.leadsV3, leads, strict=True):
      for field, value in vars(lead).items():
        setattr(builder, field, value)
    self['modelV2'] = model

  def all_checks(self):
    return True


RADAR_POINTS = [{'trackId': 7, 'dRel': 32, 'yRel': 0, 'vRel': 0}, {'trackId': 8, 'dRel': 50, 'yRel': 0, 'vRel': 0}]


def run(radard, frames):
  out = []
  for leads, points in frames:
    radard.update(RadarInputs(leads), car.RadarData.new_message(points=points))
    out.append(radard.radar_state.to_bytes())
  return out


FRAMES = [([measure(confidence=c), measure(gap=50.0, confidence=c2)], points)
          for c, c2, points in ((.9, .7, RADAR_POINTS), (.6, .45, RADAR_POINTS), (.4, .4, []), (.4, .3, []), (.9, .9, []))]


def test_radard_with_the_toggle_off_is_stock():
  off = FakeParams(**{PARAM_ENABLED: False, PARAM_CAMERA: 25, PARAM_RADAR: 25, PARAM_SMOOTH: True})
  assert run(RadarD(CP, CP_SP, params=off), FRAMES) == run(RadarD(CP, CP_SP), FRAMES)


def test_radard_hands_off_and_resets_when_settings_change():
  params = FakeParams(**{PARAM_ENABLED: True, PARAM_CAMERA: 50, PARAM_RADAR: 30, PARAM_SMOOTH: True})
  radard = RadarD(CP, CP_SP, params=params)
  run(radard, FRAMES[:2])
  assert radard.radar_state.leadOne.radar
  run(radard, FRAMES[2:3])  # radar point gone, the filtered camera confidence still admits the same car
  assert not radard.radar_state.leadOne.radar and radard.lead_handoffs[0].phase == BLEND
  params.values[PARAM_SMOOTH] = False
  radard.lead_sensitivity.read()
  run(radard, FRAMES[3:4])
  assert all(handoff.measurement is None for handoff in radard.lead_handoffs)
