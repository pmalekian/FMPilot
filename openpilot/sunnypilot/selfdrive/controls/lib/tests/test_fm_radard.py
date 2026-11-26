"""FM Pilot Lead settings through RadarD (fm-dev-sync plan PR 15).

- Fresh-install defaults are stock/off.
- With every Lead setting at its default, radarState (leadOne, leadTwo) is byte-identical to the fm-dev-sync base radard
  on randomized constructed frames, and so is the lead MPC's plan (the planner's only radarState input).
- Each setting changes what it should.
- fmRadarLead is published next to radarState with its fields filled.
"""
import math
import random
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.radard import (RADAR_TO_CAMERA, RadarD, get_custom_yrel, get_RadarState_from_vision,
                                                 laplacian_pdf)
from openpilot.sunnypilot.selfdrive.controls.lib.fm_radar import (PARAM_CAMERA, PARAM_GUARD, PARAM_RADAR, PARAM_SMOOTH,
                                                                  FmLeadConfig)

CP = SimpleNamespace(brand='rivian', flags=0)
CP_SP = SimpleNamespace(flags=0)
KEYS = (PARAM_GUARD, PARAM_RADAR, PARAM_CAMERA, PARAM_SMOOTH)


# --- the fm-dev-sync base (bb79824) radard lead selection, verbatim, as the reference ---

def stock_match_vision_to_track(v_ego, lead, tracks):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  return track if dist_sane and vel_sane else None


def stock_get_lead(v_ego, ready, tracks, lead_msg, model_v_ego, lead_prob, CP, CP_SP, low_speed_override=True):
  if len(tracks) > 0 and ready and lead_prob > .5:
    track = stock_match_vision_to_track(v_ego, lead_msg, tracks)
  else:
    track = None

  lead_dict = {'present': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_prob)
    lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)
  elif (track is None) and ready and (lead_prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
      if (not lead_dict['present']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()
  return lead_dict


class StockRadarD(RadarD):
  """RadarD with the base's update tail: the FM code never runs."""

  def update(self, sm, rr):
    self.ready = sm.seen['modelV2']
    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']
    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel] for pt in rr.points}
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)
    from openpilot.selfdrive.controls.radard import Track
    for ids in ar_pts:
      rpt = ar_pts[ids]
      v_lead = rpt[2] + self.v_ego_hist[0]
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead)
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    model_v_ego = sm['modelV2'].velocity.x[0] if len(sm['modelV2'].velocity.x) else self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      for i in range(2):
        lead_prob = leads_v3[i].prob
        if lead_prob > self.lead_prob_filters[i].x:
          self.lead_prob_filters[i].x = lead_prob
        else:
          self.lead_prob_filters[i].update(lead_prob)
      self.radar_state.leadOne = stock_get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego,
                                                self.lead_prob_filters[0].x, self.CP, self.CP_SP, low_speed_override=True)
      self.radar_state.leadTwo = stock_get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego,
                                                self.lead_prob_filters[1].x, self.CP, self.CP_SP, low_speed_override=False)


# --- constructed inputs ---

class Inputs(dict):
  def __init__(self):
    super().__init__()
    self.logMonoTime = {'modelV2': 0}
    self.recv_frame = {'carState': 0}
    self.seen = {'modelV2': True}

  def all_checks(self):
    return True


def frame(inputs, v_ego, leads, n):
  """leads: two (x, y, v, a, prob) camera leads (model frame, y right-positive)."""
  model = log.ModelDataV2.new_message()
  model.velocity.x = [v_ego]
  model.position.x, model.position.y = [0.0, 50.0, 100.0], [0.0, 0.0, 0.0]
  model.init('leadsV3', 2)
  for builder, (x, y, v, a, prob) in zip(model.leadsV3, leads, strict=True):
    builder.x, builder.y, builder.v, builder.a = [x], [y], [v], [a]
    builder.xStd, builder.yStd, builder.vStd, builder.aStd = [2.0], [1.0], [1.5], [1.0]
    builder.prob = prob
  inputs['modelV2'] = model
  inputs['carState'] = SimpleNamespace(vEgo=v_ego)
  inputs.recv_frame['carState'] = n
  inputs.logMonoTime['modelV2'] = n


def random_drive(seed, frames=300):
  """A lead car that comes and goes, radar points that come and go (incl. stationary ones and a crawl-speed stretch),
  camera confidence sweeping 0.2-0.99 across the 25-50% cutoffs."""
  rng = random.Random(seed)
  out = []
  v_ego, gap, v_lead = rng.uniform(0, 25), rng.uniform(8, 80), rng.uniform(0, 25)
  for n in range(frames):
    if n % 60 == 0:
      v_ego, gap, v_lead = rng.uniform(0, 25), rng.uniform(8, 80), rng.uniform(0, 25)
    v_ego = max(0.0, v_ego + rng.uniform(-0.3, 0.3))
    v_lead = max(0.0, v_lead + rng.uniform(-0.3, 0.3))
    gap = max(3.0, gap + (v_lead - v_ego) * DT_MDL)
    prob = min(0.99, max(0.2, 0.6 + 0.4 * math.sin(n / 7 + seed)))
    leads = [(gap + RADAR_TO_CAMERA, rng.uniform(-0.5, 0.5), v_lead, rng.uniform(-2, 1), prob),
             (gap + 25, rng.uniform(-2, 2), v_lead + 1, 0.0, prob * 0.7)]
    points = []
    if rng.random() < 0.8:
      points.append({'trackId': 1 + n // 40, 'dRel': gap + rng.uniform(-1, 1), 'yRel': rng.uniform(-0.5, 0.5),
                     'vRel': v_lead - v_ego})
    if rng.random() < 0.4:
      points.append({'trackId': 900 + n // 30, 'dRel': gap + rng.uniform(-3, 3), 'yRel': rng.uniform(-1, 1), 'vRel': -v_ego})
    if rng.random() < 0.3:
      points.append({'trackId': 500, 'dRel': rng.uniform(4, 15), 'yRel': 0.1, 'vRel': -v_ego})
    out.append((v_ego, leads, points))
  return out


def run(radards, drive):
  inputs = Inputs()
  states = [[] for _ in radards]
  for n, (v_ego, leads, points) in enumerate(drive, start=1):
    frame(inputs, v_ego, leads, n)
    rr = car.RadarData.new_message(points=points)
    for rd, out in zip(radards, states, strict=True):
      rd.update(inputs, rr)
      out.append(rd.radar_state.to_bytes())
  return states


def lead_one(raw):
  with log.RadarState.from_bytes(raw) as radar_state:
    return radar_state.leadOne.present, radar_state.leadOne.radar


class DictParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key, return_default=False):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key))


# --- defaults ---

def test_fresh_install_defaults_are_stock():
  with OpenpilotPrefix():
    params = Params()
    assert params.get(PARAM_GUARD, return_default=True) is False
    assert params.get(PARAM_RADAR, return_default=True) == 50
    assert params.get(PARAM_CAMERA, return_default=True) == 50
    assert params.get(PARAM_SMOOTH, return_default=True) is False
    for key in KEYS:  # what manager writes on first boot
      default = params.get_default_value(key)
      if default is not None:
        params.put(key, default)
    rd = RadarD(CP, CP_SP, 0.0, params)
    assert rd.fm_radar.guard is False and rd.fm_radar.config == FmLeadConfig(.5, .5, False)


# --- all off = the base ---

@pytest.mark.parametrize('seed', range(6))
def test_all_off_radar_state_is_identical_to_the_base(seed):
  drive = random_drive(seed)
  with OpenpilotPrefix():
    params = Params()
    for key in KEYS:
      params.put(key, params.get_default_value(key))
    stock, unset, defaults, written = (StockRadarD(CP, CP_SP), RadarD(CP, CP_SP), RadarD(CP, CP_SP, 0.0, params),
                                       RadarD(CP, CP_SP, 0.0, DictParams({PARAM_RADAR: 50, PARAM_CAMERA: 50})))
    base, *fm = run([stock, unset, defaults, written], drive)
  leads = [lead_one(b) for b in base]
  present, radar = sum(p for p, _ in leads), sum(r for _, r in leads)
  assert 0 < radar < present < len(base)  # the drive exercises radar, camera-only and no-lead frames
  for states in fm:
    assert states == base


def test_all_off_lead_mpc_plan_is_identical_to_the_base():
  """The planner reads radarState only through the lead MPC: same radarState in, same plan out."""
  drive = random_drive(3, frames=120)
  stock, fm = run([StockRadarD(CP, CP_SP), RadarD(CP, CP_SP, 0.0, DictParams())], drive)
  plans = []
  for states in (stock, fm):
    mpc, out = LongitudinalMpc(), []
    for raw, (v_ego, _, _) in zip(states, drive, strict=True):
      with log.RadarState.from_bytes(raw) as radar_state:
        mpc.set_cur_state(v_ego, 0.0)
        mpc.update(radar_state)
      out.append((mpc.a_solution.copy(), mpc.v_solution.copy(), int(mpc.source)))
    plans.append(out)
  for (a1, v1, s1), (a2, v2, s2) in zip(*plans, strict=True):
    assert s1 == s2 and np.array_equal(a1, a2) and np.array_equal(v1, v2)


# --- each setting's effect, through RadarD ---

def single(rd, v_ego, leads, points, inputs=None, n=1):
  inputs = inputs or Inputs()
  frame(inputs, v_ego, leads, n)
  rd.update(inputs, car.RadarData.new_message(points=points))
  return rd.radar_state.leadOne, rd.fm_lead_msg


RAMP = {'v_ego': 6.8, 'leads': [(12.7, 4.1, 8.2, 0.0, 0.99), (60.0, 0.0, 8.0, 0.0, 0.1)],
        'points': [{'trackId': 35184, 'dRel': 16.0, 'yRel': 0.8, 'vRel': -6.8}]}


def test_ignore_stationary_radar_ghosts_follows_the_camera():
  lead, _ = single(RadarD(CP, CP_SP, 0.0, DictParams()), **RAMP)
  assert lead.present and lead.radar and lead.radarTrackId == 35184
  lead, status = single(RadarD(CP, CP_SP, 0.0, DictParams({PARAM_GUARD: True})), **RAMP)
  assert lead.present and not lead.radar
  assert status.guardEnabled and status.ghostDropped and status.ghostTrackId == 35184 and status.ghostDrops == 1


@pytest.mark.parametrize('key', (PARAM_RADAR, PARAM_CAMERA))
def test_lower_confidence_admits_a_lead_sooner(key):
  far = {'v_ego': 20.0, 'leads': [(76.5, 0.0, 12.0, 0.0, 0.45), (90.0, 0.0, 12.0, 0.0, 0.1)],
         'points': [{'trackId': 3, 'dRel': 75.0, 'yRel': 0.0, 'vRel': -8.0}] if key == PARAM_RADAR else []}
  assert not single(RadarD(CP, CP_SP, 0.0, DictParams()), **far)[0].present
  lead, status = single(RadarD(CP, CP_SP, 0.0, DictParams({key: 40})), **far)
  assert lead.present and lead.radar == (key == PARAM_RADAR)
  assert status.earlyLead and status.earlyRadarMatches == (key == PARAM_RADAR)
  assert (status.radarThreshold, status.cameraThreshold) == pytest.approx((.4, .5) if key == PARAM_RADAR else (.5, .4))


def test_smooth_lead_handoffs_eases_a_farther_reading_in():
  """Radar 30 m, then only the camera at 33 m on the same car: stock jumps to 33 m, Smooth Lead Handoffs blends."""
  frames = [(12.0, [(31.52, 0.0, 12.0, 0.0, 0.9), (80.0, 0.0, 12.0, 0.0, 0.1)],
             [{'trackId': 7, 'dRel': 30.0, 'yRel': 0.0, 'vRel': 0.0}]),
            (12.0, [(34.52, 0.0, 12.0, 0.0, 0.9), (80.0, 0.0, 12.0, 0.0, 0.1)], [])]
  results = {}
  for smooth in (False, True):
    rd, inputs = RadarD(CP, CP_SP, 0.0, DictParams({PARAM_SMOOTH: smooth})), Inputs()
    for n, (v_ego, leads, points) in enumerate(frames, start=1):
      lead, status = single(rd, v_ego, leads, points, inputs, n)
    results[smooth] = (lead.dRel, str(status.decisions[0].phase), status.smoothHandoffs)
  assert results[False] == (pytest.approx(33.0), 'direct', False)
  assert 30.0 < results[True][0] < 31.0 and results[True][1:] == ('blend', True)


def test_turning_smooth_lead_handoffs_off_drops_its_state():
  params = DictParams({PARAM_SMOOTH: True})
  rd = RadarD(CP, CP_SP, 0.0, params)
  single(rd, **RAMP)
  assert rd.fm_handoffs is not None
  params.values[PARAM_SMOOTH] = False
  inputs = Inputs()
  for n in range(1, int(1 / DT_MDL) + 2):
    single(rd, RAMP['v_ego'], RAMP['leads'], RAMP['points'], inputs, n)
  assert rd.fm_handoffs is None


# --- fmRadarLead ---

class Publisher:
  def __init__(self):
    self.sent = {}

  def send(self, service, msg):
    self.sent[service] = msg.to_bytes()


def test_fm_radar_lead_is_published_with_radar_state():
  rd = RadarD(CP, CP_SP, 0.0, DictParams({PARAM_GUARD: True, PARAM_RADAR: 35, PARAM_CAMERA: 45, PARAM_SMOOTH: True}))
  single(rd, **RAMP)
  pm = Publisher()
  rd.publish(pm)
  assert set(pm.sent) == {'radarState', 'fmRadarLead'}
  with log.Event.from_bytes(pm.sent['fmRadarLead']) as event:
    assert event.valid
    m = event.fmRadarLead.as_builder()
  assert str(m.source) == 'camera' and m.leadProb == pytest.approx(0.99) and m.liveLeadProb == pytest.approx(0.99)
  assert m.ghostDropped and m.ghostTrackId == 35184 and m.ghostDRel == pytest.approx(16.0) and m.ghostDrops == 1
  assert m.guardEnabled and m.smoothHandoffs
  assert (m.radarThreshold, m.cameraThreshold) == pytest.approx((.35, .45))
  assert not m.lowSpeedLead and not m.earlyLead and m.earlyRadarMatches == 0
  assert len(m.decisions) == 2
  one, two = m.decisions
  assert str(one.source) == 'camera' and one.ghostDropped and one.ghostTrackId == 35184 and str(one.admission) == 'camera'
  assert str(two.source) == 'none' and two.confidence == pytest.approx(0.1)


def test_no_fm_radar_lead_without_model_leads():
  rd = RadarD(CP, CP_SP)
  inputs = Inputs()
  model = log.ModelDataV2.new_message()
  inputs['modelV2'], inputs['carState'] = model, SimpleNamespace(vEgo=5.0)
  rd.update(inputs, car.RadarData.new_message())
  pm = Publisher()
  rd.publish(pm)
  assert set(pm.sent) == {'radarState'}
