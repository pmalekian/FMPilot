"""FM Pilot: model icon states from the pinned Jetlink Status.icon()."""
from jetlink.openpilot.status import Status
from openpilot.selfdrive.ui.sunnypilot.onroad.fm_jetlink_icon import icon_state, stays_on


def view(**kw):
  base = Status(enabled=True, mode='usb', transport='USB', present=True, port='host', ready=True, reason=None,
                progress=None, model='Big', default_model='Big')
  return base._replace(**kw)


def state(v, started, model_seen=True, running_big=False):
  return icon_state(v, started, model_seen, running_big)


def test_parked():
  assert state(view(progress={'stage': 'download', 'frac': 0.3}), False) == 'loading'
  assert state(view(), False) == 'ready'
  assert state(view(progress={'stage': 'failed'}), False) == 'failed'
  assert state(view(present=False), False) == 'disconnected'


def test_driving():
  assert state(view(running='joining'), True) == 'loading'
  assert state(view(running='ready'), True) == 'waiting'
  assert state(view(running='running'), True, running_big=True) == 'active'
  assert state(view(running='unavailable'), True) == 'failed'
  assert state(view(running=None), True) == 'failed'


def test_only_jetlinks_green_stays_on_the_road_view():
  assert stays_on('active', True) and stays_on('waiting', True)
  assert not stays_on('active', False)                       # a chestnut's green fades as before
  assert not stays_on('loading', True) and not stays_on('failed', True)
