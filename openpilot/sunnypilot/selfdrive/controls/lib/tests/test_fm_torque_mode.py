"""FM Pilot Higher Torque: Off is AP, Headroom raises only the max request, Full scales every request, changes blend."""
from types import SimpleNamespace

import pytest

from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.fm_torque_mode import (BLEND_S, FULL, HEADROOM, OFF, PARAM_MODE, FmTorqueMode,
                                                                        high_torque_ratio)
from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP, RivianSafetyFlagsSP, max_torque_level

BLEND_STEPS = int(BLEND_S / DT_CTRL) + 2


class FakeParams:
  def __init__(self, mode=OFF):
    self.values = {PARAM_MODE: mode}

  def get(self, key, return_default=False):
    return self.values.get(key)


def make(mode=OFF, brand='rivian', panda=True, max_torque=None):
  CP = SimpleNamespace(brand=brand)
  safety_param = 0
  if panda:
    safety_param = RivianSafetyFlagsSP.HIGH_TORQUE
    if max_torque is not None:
      safety_param |= max_torque_level(max_torque) << RivianSafetyFlagsSP.TORQUE_LEVEL_SHIFT
  CP_SP = SimpleNamespace(flags=RivianFlagsSP.HIGH_TORQUE_PANDA.value if panda else 0, safetyParam=safety_param)
  params = FakeParams(mode)
  return FmTorqueMode(CP, CP_SP, params), params


def run(fm, v=4.0, steps=1):
  out = (1.0, 1.0)
  for _ in range(steps):
    out = fm.update(v)
  return out


def test_ratio_follows_the_two_curves():
  assert high_torque_ratio(0.0) == pytest.approx(481 / 385)
  assert high_torque_ratio(9.0) == pytest.approx(481 / 385)
  assert high_torque_ratio(13.0) == pytest.approx(406 / 350)
  assert high_torque_ratio(17.0) == pytest.approx(1.0)
  assert high_torque_ratio(25.0) == 1.0
  assert high_torque_ratio(35.0) == 1.0
  for v in [x * 0.25 for x in range(160)]:
    assert high_torque_ratio(v) >= 1.0


@pytest.mark.parametrize("mode", [OFF, HEADROOM, FULL])
def test_no_panda_flag_is_always_ap(mode):
  fm, _ = make(mode, panda=False)
  assert run(fm, steps=BLEND_STEPS) == (1.0, 1.0)


@pytest.mark.parametrize("mode", [OFF, HEADROOM, FULL])
def test_other_brands_are_always_ap(mode):
  fm, _ = make(mode, brand='tesla')
  assert run(fm, steps=BLEND_STEPS) == (1.0, 1.0)


def test_off_is_ap_at_every_speed():
  fm, _ = make(OFF)
  for v in (0.0, 4.0, 9.0, 13.0, 20.0, 30.0):
    assert run(fm, v=v, steps=BLEND_STEPS) == (1.0, 1.0)


def test_headroom_raises_only_the_max():
  fm, _ = make(HEADROOM)
  max_req, scale = run(fm, steps=BLEND_STEPS)
  assert max_req == pytest.approx(481 / 385)
  assert scale == 1.0
  assert run(fm, v=30.0) == (1.0, 1.0)  # highway unchanged


def test_full_scales_every_request():
  fm, _ = make(FULL)
  max_req, scale = run(fm, steps=BLEND_STEPS)
  assert max_req == 1.0
  assert scale == pytest.approx(481 / 385)


def test_mode_change_blends_over_one_second():
  fm, params = make(OFF)
  run(fm, steps=10)
  params.values[PARAM_MODE] = FULL
  fm.read_params()
  extra = 481 / 385 - 1
  prev = 1.0
  for i in range(BLEND_STEPS):
    _, scale = fm.update(4.0)
    assert scale - prev <= extra * DT_CTRL / BLEND_S + 1e-9, f"stepped at frame {i}"
    prev = scale
  assert prev == pytest.approx(1 + extra)
  # half way through a switch back, both are partly blended
  params.values[PARAM_MODE] = HEADROOM
  fm.read_params()
  max_req, scale = run(fm, steps=int(0.5 / DT_CTRL))
  assert max_req == pytest.approx(1 + extra / 2, abs=0.01)
  assert scale == pytest.approx(1 + extra / 2, abs=0.01)


def test_settings_reread_once_per_second():
  fm, params = make(OFF)
  run(fm, steps=5)
  params.values[PARAM_MODE] = HEADROOM
  run(fm, steps=int(1.0 / DT_CTRL) + BLEND_STEPS)
  assert fm.mode == HEADROOM


@pytest.mark.parametrize("bad", [None, 3, -1, "x"])
def test_bad_setting_is_off(bad):
  fm, _ = make(bad)
  assert fm.mode == OFF
  assert run(fm, steps=BLEND_STEPS) == (1.0, 1.0)



# --- Controller level: the request in carControl.actuators.torque, on every torque controller sunnypilot can pick ---
# Nov 5 drives (00000038, 00000039): Headroom and Full both stopped at exactly 1.00 because EnforceTorqueControl on with
# TorqueControlTune 0 (and EnforceTorqueControl off) run latcontrol_torque_v0, which had no Higher Torque hook.

def _rivian(panda=True):
  from opendbc.car.car_helpers import interfaces
  from opendbc.car.rivian.values import CAR
  from opendbc.car.vehicle_model import VehicleModel
  from openpilot.selfdrive.car.helpers import convert_to_capnp
  CarInterface = interfaces[CAR.RIVIAN_R1]
  CP = CarInterface.get_non_essential_params(CAR.RIVIAN_R1)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.RIVIAN_R1)
  if panda:
    CP_SP.flags |= RivianFlagsSP.HIGH_TORQUE_PANDA.value
    CP_SP.safetyParam |= RivianSafetyFlagsSP.HIGH_TORQUE  # as _initialize_rivian_high_torque sets them together
  return CP.as_reader(), convert_to_capnp(CP_SP).as_reader(), CarInterface(CP, CP_SP), VehicleModel(CP)


def _controllers():
  from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque as V1
  from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as V0
  return {"v1": V1, "v0": V0}


def _controller(version, mode, panda=True):
  CP, CP_SP, CI, VM = _rivian(panda)
  lac = _controllers()[version](CP, CP_SP, CI, DT_CTRL)
  lac.fm_torque = FmTorqueMode(CP, CP_SP, FakeParams(mode))
  return lac, VM


def _drive(lac, VM, v_ego, curvature, frames=300):
  from opendbc.car.structs import car
  CS = car.CarState(vEgo=v_ego, steeringAngleDeg=0.0)
  lp = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  out = 0.0
  for _ in range(frames):
    out, _, _ = lac.update(True, CS, VM, lp, False, curvature, None, False, 0.2)
  return out


@pytest.mark.parametrize("version", ["v0", "v1"])
@pytest.mark.parametrize("mode", [HEADROOM, FULL])
def test_controllers_go_past_one_at_low_speed(version, mode):
  lac, VM = _controller(version, mode)
  out = _drive(lac, VM, 4.0, -0.1)  # tight left turn at 9 mph, far past the cap
  assert out == pytest.approx(481 / 385, abs=1e-3), f"{version} stuck at {out}"


@pytest.mark.parametrize("version", ["v0", "v1"])
def test_controllers_off_no_panda_and_highway_stay_at_one(version):
  lac, VM = _controller(version, OFF)
  assert _drive(lac, VM, 4.0, -0.1) == pytest.approx(1.0)
  lac, VM = _controller(version, FULL, panda=False)
  assert _drive(lac, VM, 4.0, -0.1) == pytest.approx(1.0)
  lac, VM = _controller(version, HEADROOM)
  assert abs(_drive(lac, VM, 30.0, -0.01)) == pytest.approx(1.0)


@pytest.mark.parametrize("version", ["v0", "v1"])
def test_controllers_full_scales_small_requests(version):
  base = _drive(*_controller(version, OFF), 4.0, -0.002)
  full = _drive(*_controller(version, FULL), 4.0, -0.002)
  assert 0 < base < 0.9
  assert full == pytest.approx(base * 481 / 385, rel=1e-3)


@pytest.mark.parametrize("enforce,tune", [(False, 0.0), (False, 1.0), (True, 0.0), (True, 1.0)])
def test_every_selectable_torque_controller_has_the_hook(enforce, tune):
  from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
  CP, CP_SP, CI, _ = _rivian()
  vals = {"EnforceTorqueControl": enforce, "TorqueControlTune": tune}
  host = SimpleNamespace(CP=CP, CP_SP=CP_SP,
                         params=SimpleNamespace(get_bool=lambda k: bool(vals[k]), get=lambda k, **kw: vals[k]))
  chosen = ControlsExt.initialize_lateral_control(host, _controllers()["v1"](CP, CP_SP, CI, DT_CTRL), CI, DT_CTRL)
  assert isinstance(getattr(chosen, "fm_torque", None), FmTorqueMode), type(chosen).__module__
  assert chosen.fm_torque.available


@pytest.mark.parametrize("max_torque", [405, 480, 505, 580])
def test_max_torque_sets_the_headroom(max_torque):
  """FM Pilot Max Torque: Headroom's max request at low speed is the drive's cap over AP's 385."""
  fm, _ = make(HEADROOM, max_torque=max_torque)
  assert fm.cap == max_torque
  max_req, scale = run(fm, v=4.0, steps=BLEND_STEPS)
  assert max_req == pytest.approx(max_torque / 385)
  assert scale == 1.0
  assert high_torque_ratio(30.0, max_torque) == 1.0


def test_no_level_keeps_481():
  fm, _ = make(HEADROOM)
  assert fm.cap == 481
