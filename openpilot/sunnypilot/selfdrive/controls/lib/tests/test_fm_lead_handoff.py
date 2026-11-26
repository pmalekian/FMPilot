"""FM Pilot Smooth Lead Handoffs and the split lead confidence in get_lead. Ported from fm-dev-nav-settle (ca08aac9);
the Lead Shadow Log test and the old shared-cutoff migration test were dropped with those features."""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.radard import get_lead
from openpilot.sunnypilot.selfdrive.controls.lib.fm_lead_handoff import BLEND_FRAMES, HOLD_FRAMES, FmLeadHandoff
from openpilot.sunnypilot.selfdrive.controls.lib.fm_radar import (
  ADMIT_CAMERA, ADMIT_LOW_SPEED, ADMIT_RADAR, BLEND, DIRECT, HOLD, PARAM_CAMERA, PARAM_RADAR, PARAM_SMOOTH,
  FmLeadConfig, FmLeadTrace, FmRadarSettings,
)
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_fm_radar import CP, CP_SP, lead_msg, track

EGO = 12.0
PATH = SimpleNamespace(x=[0.0, 100.0], y=[0.0, 0.0])


def measure(gap=32.0, speed=12.0, lateral=0.0, confidence=0.9, accel=0.0):
  return lead_msg(x=gap + 1.52, v=speed, y=-lateral, a=accel, prob=confidence)


def select(camera, tracks=(), *, radar=0.30, vision=0.50, handoff=None, slot=0, v_ego=EGO, guard=True, probability=None,
           model_path=PATH):
  trace = FmLeadTrace()
  lead = get_lead(v_ego, True, {t.identifier: t for t in tracks}, camera, v_ego,
                  camera.prob if probability is None else probability, CP, CP_SP, low_speed_override=slot == 0,
                  stationary_guard=guard, lead_threshold=radar, fm_trace=trace, camera_threshold=vision,
                  handoff=handoff, model_path=model_path)
  return lead, trace


@pytest.mark.parametrize('slot', (0, 1))
@pytest.mark.parametrize(('radar', 'camera', 'p', 'has_track', 'source', 'admission'), (
  (.30, .50, .30, True, 'none', 0),
  (.30, .50, .40, True, 'radar', ADMIT_RADAR),
  (.30, .50, .40, False, 'none', 0),
  (.30, .50, .50, False, 'none', 0),
  (.30, .50, .51, False, 'camera', ADMIT_CAMERA),
  (.50, .40, .40, True, 'none', 0),
  (.50, .40, .45, True, 'radar', ADMIT_CAMERA),
  (.50, .40, .45, False, 'camera', ADMIT_CAMERA),
  (.50, .50, .50, True, 'none', 0),
  (.50, .50, .51, True, 'radar', ADMIT_RADAR),
))
def test_split_admission(slot, radar, camera, p, has_track, source, admission):
  points = [track(7, 32.0, 0.0, EGO, EGO)] if has_track else []
  lead, trace = select(measure(confidence=p), points, radar=radar, vision=camera, slot=slot)
  assert ('radar' if lead.get('radar') else 'camera' if lead['present'] else 'none') == source
  assert trace.admission == admission


def test_guard_fallback_requires_the_camera_cutoff():
  ghost = [track(7, 32.0, 0.0, 0.0, EGO)]
  lead, trace = select(measure(speed=8.0, confidence=.45), ghost)
  assert not lead['present'] and trace.ghost[3] == 7
  lead, trace = select(measure(speed=8.0, confidence=.55), ghost)
  assert lead['present'] and not lead['radar'] and trace.ghost[3] == 7


@pytest.mark.parametrize('slot', (0, 1))
def test_equal_cutoffs_legacy_output_at_exact_edge_and_after_loss(slot):
  candidate = track(7, 32.0, 0.0, 11.0, EGO)
  ghost = track(8, 32.0, 0.0, 0.0, EGO)
  for confidence, points in ((.5, [candidate]), (.51, [candidate]), (.8, []), (.9, [ghost]), (.9, [candidate])):
    camera = measure(confidence=confidence, speed=8.0)
    patched, trace = select(camera, points, radar=.5, vision=.5, slot=slot)
    legacy_call = get_lead(EGO, True, {point.identifier: point for point in points}, camera, EGO, confidence, CP, CP_SP,
                           low_speed_override=slot == 0, stationary_guard=True)
    assert patched == legacy_call and trace.phase == DIRECT


@pytest.mark.parametrize('slot', (0, 1))
def test_low_speed_override_keeps_raw_radar_lead_and_clears_handoff(slot):
  low_speed_point = track(7, 5.0, 0.0, 0.0, 3.0)
  handoff = FmLeadHandoff()
  select(measure(), [track(1, 32.0, 0.0, EGO, EGO)], handoff=handoff)
  lead, trace = select(measure(confidence=.10), [low_speed_point], handoff=handoff, slot=slot, v_ego=3.0)
  assert lead['present'] == (slot == 0)
  assert (trace.admission == ADMIT_LOW_SPEED) == (slot == 0)
  assert trace.phase == DIRECT and handoff.measurement is None


@pytest.mark.parametrize('radar_first', (True, False))
@pytest.mark.parametrize('change', (-1.0, 1.0))
def test_same_car_switch_is_asymmetric_for_both_sensors(radar_first, change):
  handoff = FmLeadHandoff()
  radar = track(7, 32.0 + (0 if radar_first else change), 0.0, EGO + (0 if radar_first else change), EGO)
  camera = measure(gap=32.0 + (change if radar_first else 0), speed=EGO + (change if radar_first else 0))
  before, _ = select(camera, [radar] if radar_first else [], handoff=handoff)
  after, trace = select(camera, [] if radar_first else [radar], handoff=handoff)
  assert trace.phase == BLEND and after['radar'] != before['radar']
  expected = change if change < 0 else change / BLEND_FRAMES
  assert after['dRel'] == pytest.approx(before['dRel'] + expected)
  assert after['vLead'] == pytest.approx(before['vLead'] + expected)
  assert after['vRel'] == pytest.approx(before['vRel'] + expected)


def test_blend_finishes_and_does_not_lag_further_same_source_updates():
  handoff = FmLeadHandoff()
  select(measure(), [track(7, 32.0, 0.0, EGO, EGO)], handoff=handoff)
  for _ in range(BLEND_FRAMES):
    lead, _ = select(measure(34.0), handoff=handoff)
  assert lead['dRel'] == pytest.approx(34.0)
  lead, trace = select(measure(35.0), handoff=handoff)
  assert lead['dRel'] == pytest.approx(35.0) and trace.phase == DIRECT


def test_cut_in_resets_hysteresis_and_the_previous_cars_state():
  handoff = FmLeadHandoff()
  select(measure(), [track(7, 32.0, 0.0, EGO, EGO)], handoff=handoff)
  out, trace = select(measure(gap=15.0, speed=6.0, confidence=.46), handoff=handoff)
  assert not out['present'] and handoff.estimate is None
  cut_in = measure(gap=15.0, speed=6.0, confidence=.70, accel=-2.5)
  out, trace = select(cut_in, handoff=handoff)
  assert trace.phase == DIRECT and out['dRel'] == 15.0 and out['aLeadK'] == -2.5


def test_lead_braking_during_blend_passes_through_on_the_first_frame():
  handoff = FmLeadHandoff()
  select(measure(), [track(7, 32.0, 0.0, EGO, EGO)], handoff=handoff)
  select(measure(34.0, 13.0), handoff=handoff)
  out, trace = select(measure(31.5, 10.0, accel=-3.0), handoff=handoff)
  assert trace.phase == BLEND
  assert (out['dRel'], out['vLead'], out['aLeadK']) == pytest.approx((31.5, 10.0, -3.0))


def test_hold_predicts_gap_and_times_out_without_camera_admission():
  handoff = FmLeadHandoff()
  select(measure(speed=10.0), [track(7, 32.0, 0.0, 10.0, EGO)], handoff=handoff)
  for i in range(1, HOLD_FRAMES + 1):
    lead, trace = select(measure(32.0 - i * .1, speed=10.0, confidence=.40), handoff=handoff)
    assert trace.phase == HOLD and lead['dRel'] == pytest.approx(32.0 - i * .1)
    assert lead['radarTrackId'] == 7
  lead, trace = select(measure(31.0, speed=10.0, confidence=.40), handoff=handoff)
  assert not lead['present'] and trace.phase == DIRECT


@pytest.mark.parametrize('end', ('low confidence', 'leaving path', 'ghost reject'))
def test_hold_releases_immediately(end):
  handoff = FmLeadHandoff()
  select(measure(speed=8.0), [track(7, 32.0, 0.0, 8.0, EGO)], handoff=handoff)
  assert select(measure(speed=8.0, confidence=.40), handoff=handoff)[1].phase == HOLD
  wrong_path = SimpleNamespace(x=[0.0, 100.0], y=[3.0, 3.0])
  camera = measure(speed=8.0, confidence=.01 if end == 'low confidence' else .40)
  points = [track(8, 32.0, 0.0, 0.0, EGO)] if end == 'ghost reject' else []
  lead, trace = select(camera, points, handoff=handoff, model_path=wrong_path if end == 'leaving path' else PATH)
  assert not lead['present'] and trace.phase == DIRECT and handoff.estimate is None
  assert (trace.ghost is not None) == (end == 'ghost reject')


def test_camera_clearing_its_cutoff_blends_from_hold():
  handoff = FmLeadHandoff()
  select(measure(), [track(7, 32.0, 0.0, EGO, EGO)], handoff=handoff)
  assert select(measure(confidence=.40), handoff=handoff)[1].phase == HOLD
  lead, trace = select(measure(33.0, confidence=.55), handoff=handoff)
  assert trace.phase == BLEND and not lead['radar']
  assert lead['dRel'] == pytest.approx(32.0 + 1 / BLEND_FRAMES)


@pytest.mark.parametrize('radar', (False, True))
def test_release_cutoff_is_strict_and_same_car_only(radar):
  handoff = FmLeadHandoff()
  points = [track(7, 32.0, 0.0, EGO, EGO)] if radar else []
  options = {'radar': .50, 'vision': .50, 'handoff': handoff}
  assert select(measure(confidence=.51), points, **options)[0]['present']
  lead, trace = select(measure(confidence=.46), points, **options)
  assert lead['present'] and trace.hysteresis
  assert not select(measure(confidence=.45), points, **options)[0]['present']
  assert not select(measure(confidence=.46), points, **options)[0]['present']


def test_sticky_track_yields_when_it_no_longer_matches_the_car():
  handoff = FmLeadHandoff()
  original = track(7, 32.0, 0.0, EGO, EGO)
  alternate = track(8, 33.0, 0.0, EGO, EGO)
  select(measure(), [original], handoff=handoff)
  assert select(measure(33.0), [original, alternate], handoff=handoff)[0]['radarTrackId'] == 7
  moved_off_path = track(7, 32.0, 3.0, EGO, EGO)
  assert select(measure(33.0), [moved_off_path, alternate], handoff=handoff)[0]['radarTrackId'] == 8


@pytest.mark.parametrize(('far', 'near'), ((40.0, 30.0), (95.0, 80.0), (120.0, 100.0)))
def test_far_sticky_track_yields_to_a_closer_cut_in(far, near):
  # sane() allows 25% of the gap, so at long range the followed point used to pass while the camera had already
  # moved to a car 15-20 m closer
  v_ego, path = 30.0, SimpleNamespace(x=[0.0, 200.0], y=[0.0, 0.0])
  handoff = FmLeadHandoff()
  followed = track(1, far, 0.0, 28.0, v_ego)
  for _ in range(5):
    select(measure(far, 28.0), [followed], handoff=handoff, v_ego=v_ego, model_path=path)
  cut_in = track(2, near, 0.2, 27.0, v_ego)
  lead = select(measure(near, 27.0, lateral=0.2), [followed, cut_in], handoff=handoff, v_ego=v_ego, model_path=path)[0]
  assert lead['radarTrackId'] == 2 and lead['dRel'] == near


class SettingsParams:
  def __init__(self, values):
    self.values = values

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.get(key))

  def put(self, key, value, block=False):
    assert block
    self.values[key] = value


def test_polling_picks_up_changed_choices():
  params = SettingsParams({PARAM_RADAR: 30, PARAM_CAMERA: 40})
  settings = FmRadarSettings(params)
  assert settings.config == FmLeadConfig(.30, .40)
  params.values.update({PARAM_CAMERA: 50, PARAM_SMOOTH: True})
  settings.update()
  assert settings.config == FmLeadConfig(.30, .50, True)


@pytest.mark.parametrize('value', (None, 'broken', 'inf', '-inf'))
def test_missing_or_invalid_params_use_stock_settings(value):
  params = SettingsParams({PARAM_RADAR: value, PARAM_CAMERA: value})
  assert FmRadarSettings(params).config == FmLeadConfig(.5, .5)
