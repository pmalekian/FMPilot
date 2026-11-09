"""FM Pilot: the model icon's state for Jetlink comes from jetlink's own Status.icon()."""
from jetlink.openpilot.status import Status
from openpilot.selfdrive.ui.sunnypilot.onroad.fm_jetlink_icon import icon_state, stays_on
from openpilot.selfdrive.ui.ui_state import ChestnutState


def view(**kw):
  base = Status(enabled=True, mode='usb', transport='USB', present=True, port='host', ready=True, reason=None,
                progress=None, model='Big', default_model='Big')
  return base._replace(**kw)


def state(v, started, model_seen=True, running_big=False):
  return ChestnutState(icon_state(v, started, model_seen, running_big))  # every answer is a ChestnutState


def test_parked():
  assert state(view(progress={'stage': 'download', 'frac': 0.3}), False) == ChestnutState.LOADING  # pulsing
  assert state(view(), False) == ChestnutState.READY                                                # green
  assert state(view(progress={'stage': 'failed'}), False) == ChestnutState.FAILED                   # orange
  assert state(view(present=False), False) == ChestnutState.DISCONNECTED                             # no icon


def test_driving():
  assert state(view(running='joining'), True) == ChestnutState.LOADING                    # pulsing
  assert state(view(running='ready'), True) == ChestnutState.WAITING                      # dimmed: disengage to switch
  assert state(view(running='running'), True, running_big=True) == ChestnutState.ACTIVE   # green
  assert state(view(running='unavailable'), True) == ChestnutState.FAILED                 # orange
  assert state(view(running=None), True) == ChestnutState.FAILED                          # no live modeld record


def test_only_jetlinks_green_stays_on_the_road_view():
  assert stays_on('active', True) and stays_on('waiting', True)
  assert not stays_on('active', False)                       # a chestnut's green fades as before
  assert not stays_on('loading', True) and not stays_on('failed', True)
