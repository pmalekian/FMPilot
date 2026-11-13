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


def test_turn_assist_and_low_speed_logs_are_dropped():
  msg = custom.CarControlSP.new_message()
  msg.turnAssist.sign = -1
  msg.turnAssist.holdFloor = 0.02
  msg.fmLowSpeed.feedforward = 0.3
  msg.mads.enabled = True
  cc = convert_carControlSP(msg.as_reader())
  assert cc.mads.enabled
  assert not hasattr(cc, "turnAssist") and not hasattr(cc, "fmLowSpeed")
