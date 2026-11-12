"""FM Pilot Calmer Low-Speed Correction: scales the torque controllers' low-speed proportional gain, Rivian only."""
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.fm_low_speed_gain import (BLEND_S, PARAM_ENABLED, PARAM_PERCENT, SPEEDS,
                                                                           FmLowSpeedGain, scaled_kp_table)

BLEND_STEPS = int(BLEND_S / DT_CTRL) + 2
SPEEDS_V0 = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KPS_V0 = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, 1.0]


class FakeParams:
  def __init__(self, enabled=True, percent=70):
    self.values = {PARAM_ENABLED: enabled, PARAM_PERCENT: percent}

  def get_bool(self, key):
    return bool(self.values.get(key))

  def get(self, key, return_default=False):
    return self.values.get(key)


def make(enabled=True, percent=70, brand="rivian"):
  params = FakeParams(enabled, percent)
  return FmLowSpeedGain(SimpleNamespace(brand=brand), params), params


def test_default_starts_at_the_setting():
  g, _ = make()
  assert g.update() == pytest.approx(0.7)


@pytest.mark.parametrize("enabled,percent,expected", [(False, 70, 1.0), (True, 100, 1.0), (True, 40, 0.4), (True, 10, 0.4),
                                                      (True, 500, 1.0), (True, "x", 1.0), (True, None, 0.7)])
def test_setting(enabled, percent, expected):
  g, _ = make(enabled, percent)
  assert g.update() == pytest.approx(expected)


def test_other_brands_unchanged():
  g, _ = make(brand="toyota")
  assert g.update() == 1.0


def test_change_mid_drive_blends_over_a_second():
  g, params = make(percent=70)
  params.values[PARAM_PERCENT] = 100
  g.read_params()
  assert g.update() == pytest.approx(0.7 + DT_CTRL / BLEND_S)
  for _ in range(BLEND_STEPS):
    g.update()
  assert g.scale == pytest.approx(1.0)


def test_kp_table():
  bp, kp = scaled_kp_table(SPEEDS_V0, KPS_V0, 0.7)
  def at(v):
    return float(np.interp(v, bp, kp))
  def base(v):
    return float(np.interp(v, SPEEDS_V0, KPS_V0))
  for v in (1.0, 2.0, 3.0, 5.0, SPEEDS[0]):
    assert at(v) == pytest.approx(0.7 * base(v))
  assert at(8.0) == pytest.approx(0.85 * base(8.0), rel=0.02)  # between breakpoints: linear, not an exact product
  for v in (SPEEDS[1], 10.0, 15.0, 30.0):
    assert at(v) == pytest.approx(base(v))
  assert scaled_kp_table(SPEEDS_V0, KPS_V0, 1.0)[1] == pytest.approx([base(v) for v in scaled_kp_table(SPEEDS_V0, KPS_V0, 1.0)[0]])


# --- Controller level, on both torque controllers sunnypilot can pick (v0 is what runs with Frankie's settings) ---

def _rivian():
  from opendbc.car.car_helpers import interfaces
  from opendbc.car.rivian.values import CAR
  from opendbc.car.vehicle_model import VehicleModel
  from openpilot.selfdrive.car.helpers import convert_to_capnp
  CarInterface = interfaces[CAR.RIVIAN_R1]
  CP = CarInterface.get_non_essential_params(CAR.RIVIAN_R1)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.RIVIAN_R1)
  return CP.as_reader(), convert_to_capnp(CP_SP).as_reader(), CarInterface(CP, CP_SP), VehicleModel(CP)


def _controllers():
  from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque as V1
  from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as V0
  return {"v1": V1, "v0": V0}


def _p_term(version, enabled, v_ego, frames=40):
  from opendbc.car.structs import car
  CP, CP_SP, CI, VM = _rivian()
  lac = _controllers()[version](CP, CP_SP, CI, DT_CTRL)
  lac.fm_low_speed = FmLowSpeedGain(SimpleNamespace(brand="rivian"), FakeParams(enabled, 70))
  CS = car.CarState(vEgo=v_ego, steeringAngleDeg=0.0)
  lp = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  for _ in range(frames):
    lac.update(True, CS, VM, lp, False, -0.002, None, False, 0.2)
  return lac.pid.p


@pytest.mark.parametrize("version", ["v0", "v1"])
def test_controllers_scale_the_low_speed_p_term(version):
  off, on = _p_term(version, False, 3.0), _p_term(version, True, 3.0)
  assert off != 0.0
  assert on == pytest.approx(0.7 * off, rel=1e-3)


@pytest.mark.parametrize("version", ["v0", "v1"])
def test_controllers_unchanged_from_20_mph(version):
  assert _p_term(version, True, 12.0) == pytest.approx(_p_term(version, False, 12.0))


@pytest.mark.parametrize("enforce,tune", [(False, 0.0), (True, 0.0), (True, 1.0)])
def test_every_selectable_torque_controller_has_the_hook(enforce, tune):
  from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
  CP, CP_SP, CI, _ = _rivian()
  vals = {"EnforceTorqueControl": enforce, "TorqueControlTune": tune}
  host = SimpleNamespace(CP=CP, CP_SP=CP_SP,
                         params=SimpleNamespace(get_bool=lambda k: bool(vals[k]), get=lambda k, **kw: vals[k]))
  chosen = ControlsExt.initialize_lateral_control(host, _controllers()["v1"](CP, CP_SP, CI, DT_CTRL), CI, DT_CTRL)
  assert isinstance(getattr(chosen, "fm_low_speed", None), FmLowSpeedGain), type(chosen).__module__
  assert chosen.fm_low_speed.available
