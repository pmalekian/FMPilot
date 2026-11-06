"""FM Pilot torque readout: the car controller reports the limit in force now and this drive's Max Torque, and
carstate publishes them with the EPAS ToiActive bit in carStateSP."""
import sys
import types
import unittest
from types import SimpleNamespace
from unittest import mock

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.rivian.carcontroller import CarController
from opendbc.car.rivian.values import CAR, DBC
from opendbc.sunnypilot.car.interfaces import _initialize_rivian_high_torque


def _controller(params=None):
  CP = structs.CarParams(brand='rivian')
  CP_SP = structs.CarParamsSP()
  if params:
    _initialize_rivian_high_torque(CP, CP_SP, params)
  return CarController({"pt": "rivian_primary_actuator"}, structs.CarParams(), CP_SP)


def _step(controller, v_ego, angle=0.0, lat_active=True):
  out = structs.CarState()
  out.vEgoRaw = v_ego
  out.steeringAngleDeg = angle
  cs = SimpleNamespace(out=out, acm_lka_hba_cmd={"ACM_hbaSysState": 0, "ACM_hbaLamp": 0, "ACM_hbaOnOffState": 0,
                                                 "ACM_slifOnOffState": 0},
                       sccm_wheel_touch={"SCCM_WheelTouch_Counter": 0, "SCCM_WheelTouch_HandsOn": 0,
                                         "SCCM_WheelTouch_CapacitiveValue": 0, "SCCM_WheelTouch_Calibration": 100,
                                         "SCCM_WheelTouch_ResistiveValue": 0},
                       vdm_adas_status=[], toi_fault=False, torque_tx_refused=False, toi_active=True)
  cc = structs.CarControl()
  cc.latActive = lat_active
  controller.update(cc.as_reader(), structs.CarControlSP(), cs, 0)
  return cs


class TestControllerReportsLimits(unittest.TestCase):
  def test_stock_limit(self):
    cs = _step(_controller(), 2.0)
    self.assertEqual((cs.fm_steer_cap, cs.fm_max_torque), (385, 385))

  def test_max_torque_slider_and_speed_curve(self):
    c = _controller({"FM_PandaHighTorque": True, "FM_MaxTorque": 530})
    cs = _step(c, 2.0)
    self.assertEqual((cs.fm_steer_cap, cs.fm_max_torque), (530, 530))
    self.assertEqual(_step(c, 30.0).fm_steer_cap, 275, "highway limit unchanged")
    self.assertEqual(_step(c, 30.0).fm_max_torque, 530, "the slider level is shown at any speed")

  def test_high_angle_cap_shown_unless_full_cap(self):
    c = _controller({"FM_PandaHighTorque": True, "FM_MaxTorque": 530})
    self.assertEqual(_step(c, 2.0, angle=120.0).fm_steer_cap, round(530 * 0.95))
    c = _controller({"FM_PandaHighTorque": True, "FM_MaxTorque": 530, "FM_TorqueFullCapHighAngle": True})
    self.assertEqual(_step(c, 2.0, angle=120.0).fm_steer_cap, 530)

  def test_reported_with_lateral_off(self):
    cs = _step(_controller(), 2.0, lat_active=False)
    self.assertEqual(cs.fm_steer_cap, 385)


class TestCarStatePublishes(unittest.TestCase):
  def setUp(self):
    stubs = {name: types.ModuleType(name) for name in ("openpilot", "openpilot.common", "openpilot.common.params",
                                                       "openpilot.sunnypilot", "openpilot.sunnypilot.mads",
                                                       "openpilot.sunnypilot.mads.helpers")}
    stubs["openpilot.common.params"].Params = object
    stubs["openpilot.sunnypilot.mads.helpers"].MadsSteeringModeOnBrake = object
    stubs["openpilot.sunnypilot.mads.helpers"].read_steering_mode_param = lambda *a, **k: None
    with mock.patch.dict(sys.modules, stubs):
      from opendbc.car.rivian.carstate import CarState
      from opendbc.sunnypilot.car.rivian.carstate_ext import CarStateExt
    for name in ("__init__", "update"):
      patcher = mock.patch.object(CarStateExt, name, lambda *a, **k: None)
      patcher.start()
      self.addCleanup(patcher.stop)
    cp = structs.CarParams(carFingerprint=CAR.RIVIAN_R1)
    cp_sp = structs.CarParamsSP()
    self.cs = CarState(cp, cp_sp)
    self.parsers = CarState.get_can_parsers(cp, cp_sp)
    self.packer = CANPacker(DBC[CAR.RIVIAN_R1][Bus.pt])

  def _frame(self, frame, toi_active):
    msg = self.packer.make_can_msg("EPAS_SystemStatus", 0, {"EPAS_SystemStatus_Counter": frame % 16,
                                                            "H_CAN_EPSS_ToiActive": toi_active, "EPAS_HandsOnLevel": 1})
    self.parsers[Bus.pt].update([(frame * 10_000_000, [msg])])
    return self.cs.update(self.parsers)[1]

  def test_fields(self):
    self.cs.fm_steer_cap, self.cs.fm_max_torque = 457, 530
    for frame in range(1, 4):
      ret_sp = self._frame(frame, 1)
    self.assertTrue(ret_sp.fmEpasToiActive)
    self.assertEqual((ret_sp.fmSteerCap, ret_sp.fmMaxTorque), (457, 530))
    self.assertFalse(self._frame(4, 0).fmEpasToiActive)


if __name__ == "__main__":
  unittest.main()
