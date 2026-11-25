import fmlog
from settle import windows


def event(kind, t):
  msg = fmlog.log.Event.new_message(logMonoTime=int(t * 1e9), valid=True)
  msg.init(kind)
  return msg


def frame(t, phase='idle', limit='none', push=0.0, k=0.0, guard=0.0):
  e = event('modelDataV2SP', t)
  m = e.modelDataV2SP
  m.fmTurnSettlePhase, m.fmTurnSettleLimit, m.fmTurnSettlePush, m.fmTurnSettle, m.fmTurnSettleGuard = phase, limit, push, k, guard
  return e.as_reader()


def test_one_window_push_centering_handed_off():
  cs = event('carState', 0.0)
  cs.carState.vEgo = 5.0
  evs = [cs.as_reader(), frame(0.0)]
  evs += [frame(1.0 + i * 0.05, 'push', push=0.01 * i, k=0.002) for i in range(20)]
  evs += [frame(2.0 + i * 0.05, 'centering', 'center', push=0.2, k=0.001) for i in range(10)]
  evs += [frame(2.5, 'handedOff', 'blendful', push=0.2, k=0.0005), frame(2.55, 'idle', 'blendful', push=0.2)]
  evs += [frame(4.0 + i * 0.05) for i in range(20)]
  ws = windows(evs, 0)
  assert len(ws) == 1
  w = ws[0]
  assert w.phases == ['push', 'centering', 'handedOff']
  assert w.limits == ['center', 'blendful']
  assert abs(w.push - 0.2) < 1e-6
  assert abs(w.peak - 0.002) < 1e-9
  assert abs(w.peak_lat - 0.002 * 25) < 1e-6


def test_curb_push_back_and_two_windows():
  evs = [frame(1.0, 'push', k=0.001), frame(1.05, 'push', 'curb', k=-0.002, guard=-0.003), frame(1.1, 'idle', 'curb')]
  evs += [frame(1.1 + i * 0.05) for i in range(1, 20)]
  evs += [frame(5.0, 'push', 'cap', push=0.4, k=0.0), frame(5.05, 'idle', 'cap', push=0.4)]
  ws = windows(evs, 0)
  assert len(ws) == 2
  assert ws[0].limits == ['curb'] and ws[0].guard < 0 and ws[0].least < 0
  assert ws[1].limits == ['cap']


def test_old_logs_have_no_windows():
  assert windows([frame(i * 0.05) for i in range(100)], 0) == []


def new_frame(t, phase='idle', limit='none', push=0.0, k=0.0, guard=0.0, queue=False):
  """An fm-dev-sync fmModel frame (Settle After Turn and Queue Guard, plan PR 13)."""
  e = fmlog.log_new.Event.new_message(logMonoTime=int(t * 1e9), valid=True)
  m = e.init('fmModel')
  m.turnSettlePhase, m.turnSettleLimit, m.turnSettlePush, m.turnSettle, m.turnSettleGuard = phase, limit, push, k, guard
  m.queueGuard = queue
  return e.as_reader()


def test_new_layout_fm_model_windows():
  cs = fmlog.log_new.Event.new_message(logMonoTime=0, valid=True)
  cs.init('carState').vEgo = 5.0
  evs = [cs.as_reader(), new_frame(0.0, queue=True)]
  evs += [new_frame(1.0 + i * 0.05, 'push', push=0.01 * i, k=0.002) for i in range(20)]
  evs += [new_frame(2.0 + i * 0.05, 'centering', 'center', push=0.2, k=0.001) for i in range(10)]
  evs += [new_frame(2.5, 'handedOff', 'blendful', push=0.2, k=0.0005), new_frame(2.55, 'idle', 'blendful', push=0.2)]
  evs += [new_frame(4.0 + i * 0.05) for i in range(20)]
  ws = windows(evs, 0)
  assert len(ws) == 1
  assert ws[0].phases == ['push', 'centering', 'handedOff'] and ws[0].limits == ['center', 'blendful']
  assert abs(ws[0].push - 0.2) < 1e-6 and abs(ws[0].peak_lat - 0.002 * 25) < 1e-6


def test_new_layout_old_and_new_give_the_same_windows():
  old = [frame(1.0, 'push', k=0.001), frame(1.05, 'push', 'curb', k=-0.002, guard=-0.003), frame(1.1, 'idle', 'curb')]
  new = [new_frame(1.0, 'push', k=0.001), new_frame(1.05, 'push', 'curb', k=-0.002, guard=-0.003),
         new_frame(1.1, 'idle', 'curb')]
  assert windows(old, 0) == windows(new, 0)


def test_new_layout_modeldatav2sp_is_not_settle():
  """On fm-dev-sync modelDataV2SP has no settle fields: it is never read as a settle frame."""
  e = fmlog.log_new.Event.new_message(logMonoTime=0, valid=True)
  e.init('modelDataV2SP')
  assert fmlog.fm_part(e.as_reader(), 'settle') is None
