"""FM Pilot: the road-edge lane change block is always the quiet alert (folded, see FM_FOLDED_DEFAULTS.md)."""
import inspect

from openpilot.cereal import custom, log
from openpilot.selfdrive.selfdrived import selfdrived
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET

EventNameSP = custom.OnroadEventSP.EventName
AudibleAlert = log.SelfdriveState.AudibleAlert
AlertStatus = log.SelfdriveState.AlertStatus
AlertSize = log.SelfdriveState.AlertSize


def test_quiet_alert_is_small_gray_and_silent():
  a = EVENTS_SP[EventNameSP.fmLaneChangeRoadEdgeQuiet][ET.WARNING]
  assert a.audible_alert == AudibleAlert.none
  assert a.alert_status == AlertStatus.normal
  assert a.alert_size == AlertSize.small
  assert a.alert_text_1 == EVENTS_SP[EventNameSP.laneChangeRoadEdge][ET.WARNING].alert_text_1


def test_selfdrived_raises_the_quiet_one():
  src = inspect.getsource(selfdrived.SelfdriveD)
  assert "OnroadEventSP.EventName.fmLaneChangeRoadEdgeQuiet" in src
  assert "OnroadEventSP.EventName.laneChangeRoadEdge)" not in src
