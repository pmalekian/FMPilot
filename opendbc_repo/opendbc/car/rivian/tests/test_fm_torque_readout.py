"""FM Pilot torque telemetry cases ported from fm-dev-nav-settle ca08aac9."""
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.rivian.carcontroller import CarController
from opendbc.sunnypilot.car.interfaces import _initialize_rivian_max_torque


@pytest.mark.parametrize("max_torque, speed, angle, lateral_active, expected_cap", [
  (385, 2., 0., True, 385),
  (385, 2., 120., True, 366),
  (385, 2., 120., False, 366),
  (535, 2., 120., True, 535),
  (535, 30., 0., True, 275),
  (580, 2., 0., True, 580),
])
def test_controller_reports_cap_in_force(max_torque, speed, angle, lateral_active, expected_cap):
  cp = structs.CarParams(brand='rivian')
  cp_sp = structs.CarParamsSP()
  _initialize_rivian_max_torque(cp, cp_sp, {"FM_CustomMaxTorque": max_torque})
  controller = CarController({"pt": "rivian_primary_actuator"}, cp, cp_sp)
  state = SimpleNamespace(out=structs.CarState(vEgoRaw=speed, steeringAngleDeg=angle),
                          acm_lka_hba_cmd={"ACM_hbaSysState": 0, "ACM_hbaLamp": 0, "ACM_hbaOnOffState": 0,
                                           "ACM_slifOnOffState": 0},
                          sccm_wheel_touch={"SCCM_WheelTouch_Counter": 0, "SCCM_WheelTouch_HandsOn": 0,
                                            "SCCM_WheelTouch_CapacitiveValue": 0, "SCCM_WheelTouch_Calibration": 100,
                                            "SCCM_WheelTouch_ResistiveValue": 0},
                          vdm_adas_status=[], toi_active=True)

  control = structs.CarControl(latActive=lateral_active)
  controller.update(control.as_reader(), structs.CarControlSP(), state, 0)

  assert (controller.fm_steer_cap, controller.fm_max_torque) == (expected_cap, max_torque)
