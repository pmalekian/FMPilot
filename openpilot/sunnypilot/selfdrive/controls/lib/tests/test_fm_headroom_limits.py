"""FM Pilot Max Torque Headroom keeps the PID's limits in the right units: lateral accel normally, torque (= the max
request) with Lateral Jerk, whose extension owns them. Found in the PR #8 review (the limit came out 3.03 instead of
1.08 at 580 with Lateral Jerk on)."""
import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.rivian.values import CAR
from opendbc.car.structs import car
from opendbc.car.vehicle_model import VehicleModel
from opendbc.sunnypilot.car.interfaces import setup_interfaces as opendbc_setup_interfaces
from openpilot.cereal import log
from openpilot.common.mock.generators import generate_deviceMotion
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.locationd.helpers import Pose
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_latcontrol_torque_ext import _make_model_v2


def make(cls, max_torque, jerk):
  params = Params()
  params.put_bool("EnforceTorqueControl", True, block=True)
  params.put_bool("LateralJerkTorqueController", jerk, block=True)
  params.put_bool("NeuralNetworkLateralControl", False, block=True)
  params.put("FM_CustomMaxTorque", max_torque, block=True)
  CarInterface = interfaces[CAR.RIVIAN_R1]
  CP = CarInterface.get_non_essential_params(CAR.RIVIAN_R1)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.RIVIAN_R1)
  CI = CarInterface(CP, CP_SP)
  # as card does: the opendbc side reads FM_CustomMaxTorque into the car flag and the safety param
  opendbc_setup_interfaces(CI, CP, CP_SP, sunnypilot_interfaces.initialize_params(params))
  sunnypilot_interfaces.setup_interfaces(CI, params)
  CP_SP = convert_to_capnp(CP_SP)
  return cls(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL), VehicleModel(CP)


def step(controller, VM, v, active=True, update_ext_limits=True):
  CS = car.CarState.new_message()
  CS.vEgo = v
  CS.vEgoRaw = v
  pose = Pose.from_device_motion(generate_deviceMotion().deviceMotion)
  if update_ext_limits:
    controller.extension.update_limits()  # as controlsd does each frame with learned torque params
  controller.extension.update_model_v2(_make_model_v2().modelV2)
  controller.extension.update_lateral_lag(0.2)
  controller.update(active, CS, VM, log.VehicleParameters.new_message(), False, 0.02, pose, False, 0.2)


CONTROLLERS = (LatControlTorque, LatControlTorqueV0)  # v1, v0


class TestHeadroomLimits(OpenpilotTestCase):
  def test_jerk_limits_follow_the_max_request(self):
    for cls in CONTROLLERS:
      controller, VM = make(cls, 580, jerk=True)
      assert controller.fm_torque.available and controller.extension._jerk_aware_enabled
      for i in range(200):  # through the 1 s blend at 5 m/s, then 15 m/s
        step(controller, VM, 5.0 if i < 150 else 15.0)
        assert controller.steer_max > 1.0, (cls, i)
        assert controller.pid.pos_limit == pytest.approx(controller.steer_max), (cls, i)
        assert controller.pid.neg_limit == pytest.approx(-controller.steer_max), (cls, i)

  def test_disengaged_startup_then_engage_keeps_torque_units(self):
    for cls in CONTROLLERS:
      controller, VM = make(cls, 580, jerk=True)
      assert controller.fm_torque.available and controller.extension._jerk_aware_enabled
      for _ in range(120):  # startup blend finishes while disengaged, without controlsd updating limits
        step(controller, VM, 0.0, active=False, update_ext_limits=False)
      assert controller.steer_max == pytest.approx(580 / 385)
      assert controller.pid.pos_limit == pytest.approx(controller.steer_max), cls
      assert controller.pid.neg_limit == pytest.approx(-controller.steer_max), cls
      for i in range(10):  # engage at 5 m/s: limits stay in torque units from frame 0
        step(controller, VM, 5.0, active=True, update_ext_limits=False)
        assert controller.pid.pos_limit == pytest.approx(controller.steer_max), (cls, i)
        assert controller.pid.neg_limit == pytest.approx(-controller.steer_max), (cls, i)

  def test_override_torque_params_keeps_torque_units(self):
    for cls in CONTROLLERS:
      params = Params()
      params.put_bool("TorqueParamsOverrideEnabled", True, block=True)
      params.put("TorqueParamsOverrideLatAccelFactor", 2.2, block=True)
      params.put("TorqueParamsOverrideFriction", 0.1, block=True)
      controller, VM = make(cls, 580, jerk=True)
      for _ in range(300):  # settle headroom before the next 300-frame override polling tick (frame 300)
        step(controller, VM, 5.0, active=True, update_ext_limits=False)
      settled_max = controller.steer_max
      assert settled_max == pytest.approx(580 / 385), cls
      assert controller.torque_params.latAccelFactor == pytest.approx(2.2), cls
      assert controller.torque_params.friction == pytest.approx(0.1), cls
      # Change override values right before polling frame 300, when headroom is already steady
      params.put("TorqueParamsOverrideLatAccelFactor", 2.8, block=True)
      params.put("TorqueParamsOverrideFriction", 0.15, block=True)
      step(controller, VM, 5.0, active=True, update_ext_limits=False)
      assert controller.steer_max == settled_max, cls
      assert controller.torque_params.latAccelFactor == pytest.approx(2.8), cls
      assert controller.torque_params.friction == pytest.approx(0.15), cls
      assert controller.pid.pos_limit == pytest.approx(controller.steer_max), cls
      assert controller.pid.neg_limit == pytest.approx(-controller.steer_max), cls

  def test_accel_limits_without_jerk(self):
    for cls in CONTROLLERS:
      controller, VM = make(cls, 580, jerk=False)
      for _ in range(120):
        step(controller, VM, 5.0)
      expected = controller.lateral_accel_from_torque(controller.steer_max, controller.torque_params)
      assert controller.steer_max > 1.0 and controller.pid.pos_limit == pytest.approx(expected), cls

  def test_385_never_touches_the_limits(self):
    for cls in CONTROLLERS:
      for jerk in (False, True):
        controller, VM = make(cls, 385, jerk=jerk)
        assert not controller.fm_torque.available
        for _ in range(50):
          step(controller, VM, 5.0)
          assert controller.steer_max == 1.0, (cls, jerk)
