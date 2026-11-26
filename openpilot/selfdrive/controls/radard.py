#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from openpilot.cereal import messaging, log, custom
from opendbc.car.structs import car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from openpilot.sunnypilot.selfdrive.controls.lib.fm_lead_handoff import (
  FmLeadHandoff, RELEASE_MARGIN, SAME_CAR_LATERAL, SAME_CAR_SPEED, camera_on_path, same_car, same_car_distance_margin,
)
from openpilot.sunnypilot.selfdrive.controls.lib.fm_radar import (
  ADMIT_CAMERA, ADMIT_LOW_SPEED, ADMIT_RADAR, FmLeadTrace, FmRadarLeadStatus, FmRadarSettings, stationary_ghost,
)


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "present": True,
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track],
                          preferred: Track | None = None):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    # This isn't exactly right, but it's a good heuristic
    return prob_d * prob_y * prob_v

  def sane(track):
    # Stationary radar points can be false positives.
    dist_sane = abs(track.dRel - offset_vision_dist) < max(offset_vision_dist * .25, 5.0)
    vel_sane = abs(track.vRel + v_ego - lead.v[0]) < 10 or v_ego + track.vRel > 3
    return dist_sane and vel_sane

  # The followed point must also be at the camera's distance: sane() allows 25% of the gap, so at long range a car
  # that cuts in 15-20 m closer could otherwise lose to the point already followed.
  if (preferred is not None and sane(preferred) and
      abs(preferred.dRel - offset_vision_dist) < same_car_distance_margin(offset_vision_dist) and
      abs(preferred.yRel + lead.y[0]) < SAME_CAR_LATERAL and
      abs(preferred.vRel + v_ego - lead.v[0]) < SAME_CAR_SPEED):
    return preferred

  track = max(tracks.values(), key=prob)
  return track if sane(track) else None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float, lead_prob: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "modelProb": float(lead_prob),
    "present": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, lead_prob: float, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
             low_speed_override: bool = True, stationary_guard: bool = False,
             lead_threshold: float = .5, fm_trace: FmLeadTrace | None = None, *,
             camera_threshold: float | None = None, handoff: FmLeadHandoff | None = None,
             model_path=None) -> dict[str, Any]:
  # FM Pilot Lead Detection Sensitivity (FrogPilot, MIT), split into radar and camera admission thresholds.
  radar_threshold = lead_threshold
  camera_threshold = radar_threshold if camera_threshold is None else camera_threshold
  match_threshold = min(radar_threshold, camera_threshold)
  trace = fm_trace if fm_trace is not None else FmLeadTrace()

  if handoff is not None:
    ready = ready and all(len(getattr(lead_msg, field)) for field in ('x', 'y', 'v', 'a', 'xStd', 'yStd', 'vStd'))

  reference = handoff.predict(handoff.measurement, v_ego) if handoff is not None else None
  preferred = None
  if reference is not None and reference['radar']:
    candidate = tracks.get(reference['radarTrackId'])
    if candidate is not None and same_car(reference, candidate.get_RadarState()):
      preferred = candidate

  lead_dict = {'present': False}
  if tracks and ready and lead_prob > match_threshold - (RELEASE_MARGIN if reference is not None else 0.0):
    track = match_vision_to_track(v_ego, lead_msg, tracks, preferred)
    if track is not None:
      radar_lead = get_custom_yrel(CP, CP_SP, track.get_RadarState(lead_prob), lead_msg)
      hysteresis = lead_prob <= match_threshold
      if not hysteresis or same_car(reference, radar_lead):
        if stationary_guard and stationary_ghost(track.vLead, lead_msg.v[0], lead_msg.a[0]):
          trace.ghost = (track.dRel, track.yRel, track.vLead, track.identifier)
        else:
          lead_dict = radar_lead
          trace.admission = ADMIT_RADAR if lead_prob > radar_threshold or (
            hysteresis and radar_threshold <= camera_threshold) else ADMIT_CAMERA
          trace.hysteresis = hysteresis

  camera = None
  if ready and not lead_dict['present'] and (handoff is not None or lead_prob > camera_threshold):
    camera = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)
    camera_hysteresis = lead_prob <= camera_threshold
    if not camera_hysteresis or (lead_prob > camera_threshold - RELEASE_MARGIN and same_car(reference, camera)):
      lead_dict = camera
      trace.admission = ADMIT_CAMERA
      trace.hysteresis = camera_hysteresis

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if low_speed_tracks:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
      if not lead_dict['present'] or closest_track.dRel < lead_dict['dRel']:
        lead_dict = closest_track.get_RadarState()
        trace.low_speed = True
        trace.admission = ADMIT_LOW_SPEED
        trace.hysteresis = False

  if handoff is not None:
    # Use unfiltered confidence for the hold: the asymmetric admission filter can outlast a lost car.
    camera_supports_hold = (not lead_dict['present'] and camera is not None and
                            lead_msg.prob > match_threshold - RELEASE_MARGIN and camera_on_path(lead_msg, model_path))
    lead_dict = handoff.update(lead_dict, camera, camera_supports_hold, v_ego, lead_prob, trace)

  return lead_dict


def get_custom_yrel(CP: structs.CarParams, CP_SP: structs.CarParamsSP, lead_dict: dict[str, Any],
                    lead_msg: capnp._DynamicStructReader) -> dict[str, Any]:
  if CP.brand == "hyundai" and (CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC or
                                CP.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC)):
    lead_dict['yRel'] = float(-lead_msg.y[0])

  return lead_dict


class RadarD:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParams, delay: float = 0.0, params=None):
    self.CP = CP
    self.CP_SP = CP_SP
    self.fm_radar = FmRadarSettings(params)  # FM Pilot radar lead settings; params=None keeps openpilot's behavior
    self.fm_lead_status = FmRadarLeadStatus()  # FM Pilot Radar Lead Status (fmRadarLead), logging only
    self.fm_lead_msg = None
    self.fm_handoffs = None  # Smooth Lead Handoffs state for leadOne and leadTwo while it is on
    self.fm_previous_settings = None

    self.current_time = 0.0
    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)
    self.lead_prob_filters = [FirstOrderFilter(0.0, 0.2, DT_MDL) for _ in range(2)]

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead)

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    self.fm_radar.update()
    self.fm_lead_msg = None
    if len(leads_v3) < 2:
      self.fm_handoffs = None
      return

    for lead, prob_filter in zip(leads_v3, self.lead_prob_filters, strict=False):
      # Asymmetric filter on lead prob to keep lead when uncertain
      if lead.prob > prob_filter.x:
        prob_filter.x = lead.prob
      else:
        prob_filter.update(lead.prob)

    # FM Pilot: any change of the lead settings starts the Smooth Lead Handoffs state over (as fm-dev-nav-settle)
    settings = (self.fm_radar.guard, self.fm_radar.config)
    if settings != self.fm_previous_settings:
      self.fm_handoffs = None
    self.fm_previous_settings = settings
    if self.fm_radar.config.smooth and self.fm_handoffs is None:
      self.fm_handoffs = [FmLeadHandoff(), FmLeadHandoff()]

    leads, traces, decisions = self.evaluate_leads(leads_v3, model_v_ego, sm['modelV2'].position)
    self.radar_state.leadOne, self.radar_state.leadTwo = leads
    self.fm_lead_msg = custom.FmRadarLead.new_message()
    self.fm_lead_status.fill(self.fm_lead_msg, leads[0], traces[0], self.fm_radar, self.lead_prob_filters[0].x)
    self.fm_lead_msg.decisions = decisions

  def evaluate_leads(self, leads_v3, model_v_ego, model_path):
    config, handoffs = self.fm_radar.config, self.fm_handoffs
    leads, traces, decisions = [], [], []
    for i, prob_filter in enumerate(self.lead_prob_filters):
      trace = FmLeadTrace()
      lead = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[i], model_v_ego, prob_filter.x, self.CP, self.CP_SP,
                      low_speed_override=i == 0, stationary_guard=self.fm_radar.guard, lead_threshold=config.radar,
                      fm_trace=trace, camera_threshold=config.camera, handoff=handoffs[i] if handoffs is not None else None,
                      model_path=model_path if handoffs is not None else None)
      leads.append(lead)
      traces.append(trace)
      decisions.append(trace.decision(lead, prob_filter.x))
    return leads, traces, decisions

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  cloudlog.info("radard is waiting for CarParamsSP")
  CP_SP = messaging.log_from_bytes(Params().get("CarParamsSP", block=True), custom.CarParamsSP)
  cloudlog.info("radard got CarParamsSP")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'radarTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP, CP_SP, CP.radarDelay, Params())

  while 1:
    sm.update()

    RD.update(sm, sm['radarTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
