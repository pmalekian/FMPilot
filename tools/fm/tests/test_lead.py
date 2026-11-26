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


def new_event(kind, t):
  msg = fmlog.log_new.Event.new_message(logMonoTime=t, valid=True)
  msg.init(kind)
  return msg


def test_fm_dev_sync_fm_radar_lead_without_shadow_is_reported(capsys):
  """fm-dev-sync's fmRadarLead has no shadow list (Lead Shadow Log dropped): only the active decisions are reported."""
  radar = new_event('radarState', 100_000_000)
  status = new_event('fmRadarLead', 101_000_000)
  m = status.fmRadarLead
  m.radarThreshold, m.cameraThreshold, m.smoothHandoffs = .35, .45, True
  one, _ = m.init('decisions', 2)
  one.source, one.admission, one.phase, one.ghostDropped = 'camera', 'camera', 'blend', True
  one.dRel, one.ghostTrackId, one.confidence = 18, 35184, .9
  plan = new_event('longitudinalPlan', 110_000_000)
  plan.longitudinalPlan.longitudinalPlanSource = 'lead0'
  plan.longitudinalPlan.aTarget = -.4

  frames, plans = shadow_report([radar.as_reader(), status.as_reader(), plan.as_reader()], 0)
  assert [f.variant for f in frames] == ['active 35%/45% on']
  assert frames[0].decisions[0]['ghostTrackId'] == 35184 and frames[0].decisions[0]['phase'] == 'blend'
  assert frames[0].decisions[1]['source'] == 'none'
  assert plans[0].accel == pytest.approx(-.4)
  summarize(frames, plans, 'active')
  output = capsys.readouterr().out
  assert "guard=1" in output and "blend=1" in output


@pytest.mark.parametrize('layout', ('old', 'new'))
def test_replay_reads_both_log_layouts(layout):
  """--replay reruns this branch's radard and planner on an fm-dev-nav-settle or fm-dev-sync drive."""
  pytest.importorskip('openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx')
  from lead import replay
  schema = fmlog.log_old if layout == 'old' else fmlog.log_new
  events = []

  def add(kind, t):
    msg = schema.Event.new_message(logMonoTime=t, valid=True)
    msg.init(kind)
    events.append(msg)
    return getattr(msg, kind)

  add('carParams', 1).brand = 'rivian'
  add('carParamsSP', 2)
  for n in range(1, 21):
    t = n * 50_000_000
    add('carState', t).vEgo = 12.0
    point = add('radarTracks', t + 1).init('points', 1)[0]
    point.trackId, point.dRel = 7, 30.0
    model = add('modelV2', t + 2)
    model.velocity.x, model.position.x, model.position.y = [12.0], [0.0, 100.0], [0.0, 0.0]
    for lead, x in zip(model.init('leadsV3', 2), (31.52, 60.0), strict=True):
      lead.x, lead.y, lead.v, lead.a, lead.prob = [x], [0.0], [12.0], [0.0], .9
      lead.xStd, lead.yStd, lead.vStd, lead.aStd = [2.0], [1.0], [1.5], [1.0]
    add('radarState', t + 3).mdMonoTime = t + 2
    add('longitudinalPlan', t + 4).modelMonoTime = t + 2

  leads, plans, guard = replay([e.as_reader() for e in events], 0)
  assert guard is False and len(plans) == 8 * 20
  stock = [f for f in leads if f.variant == '50%/50% off']
  assert len(stock) == 20 and all(f.decisions[0]['source'] == 'radar' and f.decisions[0]['radarTrackId'] == 7 for f in stock)
