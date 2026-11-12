"""FM Pilot Low-Speed Steering Feedforward: torque from the asked wheel angle below about 16 mph, Rivian only, off by
default, strength per side (Off / Gentle / Medium / Firm / Full, Gentle by default)."""
import math
from types import SimpleNamespace

import pytest

from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.fm_low_speed_ff import (BASE_COUNTS, BLEND_S, PARAM_ENABLED, PARAM_STEP_LEFT,
                                                                         PARAM_STEP_RIGHT, SPEEDS, STEPS, FmLowSpeedFF)

LAF = 2.7  # learned latAccelFactor in the Nov 11 logs
BLEND_STEPS = int(BLEND_S / DT_CTRL) + 2


class FakeParams:
  def __init__(self, enabled=True, step=1, right=None):
    self.values = {PARAM_ENABLED: enabled, PARAM_STEP_LEFT: step, PARAM_STEP_RIGHT: step if right is None else right}

  def get_bool(self, key):
    return bool(self.values.get(key))

  def get(self, key, return_default=False):
    return self.values.get(key)


def _vm():
  from opendbc.car.car_helpers import interfaces
  from opendbc.car.rivian.values import CAR
  from opendbc.car.vehicle_model import VehicleModel
  CP = interfaces[CAR.RIVIAN_R1].get_non_essential_params(CAR.RIVIAN_R1)
  return VehicleModel(CP)


VM = _vm()


def curvature_for(wheel_deg, v):
  """Right-positive curvature (controlsd's) for a left-positive wheel angle."""
  return -VM.calc_curvature(math.radians(wheel_deg), v, 0.0)


def make(enabled=True, step=1, brand="rivian", right=None):
  return FmLowSpeedFF(SimpleNamespace(brand=brand), FakeParams(enabled, step, right))


def counts(ff, wheel_deg, v, fm_scale=1.0):
  """The added feedforward back in counts, signed left-positive like the wheel angle."""
  k = curvature_for(wheel_deg, v)
  return -ff.update(k, v, VM, LAF, fm_scale) / LAF * BASE_COUNTS * fm_scale


def test_off_by_default_values():
  from openpilot.common.params import Params
  assert not Params().get_bool(PARAM_ENABLED)
  assert int(Params().get(PARAM_STEP_LEFT, return_default=True)) == 1
  assert int(Params().get(PARAM_STEP_RIGHT, return_default=True)) == 1


@pytest.mark.parametrize("step,frac", sorted(STEPS.items()))
def test_steps_both_sides(step, frac):
  ff = make(step=step)
  assert counts(ff, 30.0, 3.0) == pytest.approx(70.0 * frac, rel=0.02)  # left
  assert counts(ff, -30.0, 3.0) == pytest.approx(-70.0 * frac, rel=0.02)  # right


def test_steps_are_off_gentle_medium_firm_full():
  assert STEPS == {0: 0.0, 1: 0.4, 2: 0.6, 3: 0.8, 4: 1.0}


@pytest.mark.parametrize("left,right", [(0, 1), (1, 0), (3, 1), (1, 4)])
def test_each_side_on_its_own(left, right):
  ff = make(step=left, right=right)
  assert counts(ff, 60.0, 3.0) == pytest.approx(FmLowSpeedFF.counts(60.0, 3.0) * STEPS[left], rel=0.02)
  assert counts(ff, -60.0, 3.0) == pytest.approx(-FmLowSpeedFF.counts(60.0, 3.0) * STEPS[right], rel=0.02)


def test_gentle_is_the_default_step_and_bad_values_fall_back_to_it():
  for bad in (None, 9, -1, "x"):
    assert make(step=bad, right=bad).target == {"left": STEPS[1], "right": STEPS[1]}
  assert make(step=0, right=0).target == {"left": 0.0, "right": 0.0}  # Off is a real choice, not a bad value


def test_off_disabled_or_other_brand_is_zero():
  assert counts(make(enabled=False), 90.0, 3.0) == 0.0
  assert counts(make(brand="toyota"), 90.0, 3.0) == 0.0


@pytest.mark.parametrize("other", ["LateralJerkTorqueController", "NeuralNetworkLateralControl"])
def test_stands_down_with_controllers_that_rebuild_the_feedforward(other):
  params = FakeParams(True, 4)
  params.values[other] = True
  ff = FmLowSpeedFF(SimpleNamespace(brand="rivian"), params)
  assert not ff.available
  assert ff.update(curvature_for(60.0, 3.0), 3.0, VM, LAF) == 0.0


def test_direction_follows_the_turn():
  ff = make(step=4)
  assert counts(ff, 60.0, 3.0) > 0  # left
  assert counts(ff, -60.0, 3.0) == pytest.approx(-counts(ff, 60.0, 3.0))  # right, same size
  assert ff.update(0.0, 3.0, VM, LAF) == 0.0


def test_speed_fade():
  ff = make(step=4)
  full = counts(ff, 60.0, 2.0)
  assert counts(ff, 60.0, SPEEDS[0]) == pytest.approx(full, rel=0.02)
  assert 0 < counts(ff, 60.0, (SPEEDS[0] + SPEEDS[1]) / 2) < full
  assert counts(ff, 60.0, SPEEDS[1]) == 0.0
  assert counts(ff, 60.0, 15.0) == 0.0


def test_table_flattens_for_tight_turns():
  ff = make(step=4)
  assert counts(ff, 180.0, 2.0) == pytest.approx(110.0, rel=0.02)
  assert counts(ff, 400.0, 2.0) == pytest.approx(110.0, rel=0.02)


def test_full_torque_mode_keeps_the_counts():
  ff = make(step=4)
  assert counts(ff, 60.0, 3.0, fm_scale=1.5) == pytest.approx(counts(ff, 60.0, 3.0), rel=1e-6)


def test_change_mid_drive_blends_each_side():
  ff = make(step=1)
  ff.params.values[PARAM_STEP_LEFT] = 4
  ff.params.values[PARAM_STEP_RIGHT] = 0
  ff.read_params()
  first = counts(ff, 60.0, 3.0)
  assert counts(ff, -60.0, 3.0) < 0  # right fades out, not off at once
  for _ in range(BLEND_STEPS):
    ff.update(curvature_for(60.0, 3.0), 3.0, VM, LAF)
  assert first < counts(ff, 60.0, 3.0)
  assert ff.strength == pytest.approx({"left": 1.0, "right": 0.0})
  assert counts(ff, -60.0, 3.0) == 0.0


# --- Controller level, both torque controllers ---

def _rivian():
  from opendbc.car.car_helpers import interfaces
  from opendbc.car.rivian.values import CAR
  from openpilot.selfdrive.car.helpers import convert_to_capnp
  CarInterface = interfaces[CAR.RIVIAN_R1]
  CP = CarInterface.get_non_essential_params(CAR.RIVIAN_R1)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.RIVIAN_R1)
  return CP.as_reader(), convert_to_capnp(CP_SP).as_reader(), CarInterface(CP, CP_SP)


def _controllers():
  from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque as V1
  from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as V0
  return {"v1": V1, "v0": V0}


def _first_output(version, enabled, v, wheel_deg):
  """Output with the truck already at the asked angle (no error), after the controller's 0.2 s request delay has passed."""
  from opendbc.car.structs import car
  from openpilot.sunnypilot.selfdrive.controls.lib.fm_low_speed_gain import FmLowSpeedGain
  CP, CP_SP, CI = _rivian()
  lac = _controllers()[version](CP, CP_SP, CI, DT_CTRL)
  lac.fm_low_speed = FmLowSpeedGain(SimpleNamespace(brand="toyota"))  # keep the correction at openpilot's
  lac.fm_low_speed_ff = FmLowSpeedFF(SimpleNamespace(brand="rivian"), FakeParams(enabled, 4))
  CS = car.CarState(vEgo=v, steeringAngleDeg=wheel_deg)
  lp = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  k = curvature_for(wheel_deg, v)
  out = 0.0
  for _ in range(40):
    out, _, _ = lac.update(True, CS, VM, lp, False, k, None, False, 0.2)
  return out


@pytest.mark.parametrize("version", ["v0", "v1"])
def test_controllers_add_it(version):
  off, on = _first_output(version, False, 3.0, 60.0), _first_output(version, True, 3.0, 60.0)
  expected = FmLowSpeedFF.counts(60.0, 3.0) / BASE_COUNTS
  assert (on - off) == pytest.approx(expected, rel=0.03)  # left-positive output, a left turn


@pytest.mark.parametrize("version", ["v0", "v1"])
def test_controllers_unchanged_above_16_mph(version):
  assert _first_output(version, True, 9.0, 20.0) == pytest.approx(_first_output(version, False, 9.0, 20.0))


@pytest.mark.parametrize("enforce,tune", [(False, 0.0), (True, 0.0), (True, 1.0)])
def test_every_selectable_torque_controller_has_the_hook(enforce, tune):
  from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
  CP, CP_SP, CI = _rivian()
  vals = {"EnforceTorqueControl": enforce, "TorqueControlTune": tune}
  host = SimpleNamespace(CP=CP, CP_SP=CP_SP,
                         params=SimpleNamespace(get_bool=lambda k: bool(vals[k]), get=lambda k, **kw: vals[k]))
  chosen = ControlsExt.initialize_lateral_control(host, _controllers()["v1"](CP, CP_SP, CI, DT_CTRL), CI, DT_CTRL)
  assert isinstance(getattr(chosen, "fm_low_speed_ff", None), FmLowSpeedFF), type(chosen).__module__
  assert chosen.fm_low_speed_ff.available
