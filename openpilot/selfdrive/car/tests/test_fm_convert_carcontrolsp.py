from openpilot.cereal import custom
from openpilot.selfdrive.car.helpers import convert_carControlSP


def test_log_only_fields_are_dropped():
  # FM Pilot: releaseGuard is in carControlSP for logging only; opendbc's CarControlSP has no such field
  msg = custom.CarControlSP.new_message()
  msg.releaseGuard.active = True
  msg.releaseGuard.amount = 0.5
  msg.mads.enabled = True
  msg.laneCentering.active = True
  cc = convert_carControlSP(msg.as_reader())
  assert cc.mads.enabled
  assert cc.laneCentering.active
  assert not hasattr(cc, "releaseGuard")
