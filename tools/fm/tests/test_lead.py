import pytest

import fmlog
from lead import shadow_report, summarize


def event(kind, t):
  msg = fmlog.log.Event.new_message(logMonoTime=t, valid=True)
  msg.init(kind)
  return msg


def test_shadow_report_links_logged_plan_and_marks_candidate_only(capsys):
  radar = event('radarState', 100_000_000)
  status = event('fmRadarLead', 101_000_000)
  status.fmRadarLead.radarThreshold = status.fmRadarLead.cameraThreshold = .5
  status.fmRadarLead.init('decisions', 2)
  shadow = status.fmRadarLead.init('shadow', 1)[0]
  shadow.radarThreshold, shadow.cameraThreshold, shadow.smoothHandoffs = .30, .40, True
  lead = shadow.init('decisions', 2)[0]
  lead.source, lead.admission, lead.phase = 'radar', 'hold', 'hold'
  lead.dRel, lead.radarTrackId, lead.confidence = 22, 7, .35
  plan = event('longitudinalPlan', 110_000_000)
  plan.longitudinalPlan.longitudinalPlanSource = 'lead0'
  plan.longitudinalPlan.aTarget = -.75

  frames, plans = shadow_report([radar.as_reader(), status.as_reader(), plan.as_reader()], 0)
  assert [f.variant for f in frames] == ['active 50%/50% off', '30%/40% on']
  assert frames[1].decisions[0]['dRel'] == 22
  assert frames[1].decisions[0]['radarTrackId'] == 7
  assert frames[1].decisions[1]['radarTrackId'] == -1
  assert plans[0].lead_time == frames[1].time
  assert plans[0].accel == pytest.approx(-.75)
  summarize(frames, plans, 'active')
  output = capsys.readouterr().out
  assert "observed planner on candidate-only frames: {'lead0': 1}" in output
  assert "hold=1" in output


def test_old_fm_radar_lead_messages_are_skipped():
  status = event('fmRadarLead', 100_000_000)
  assert shadow_report([status.as_reader()], 0) == ([], [])
