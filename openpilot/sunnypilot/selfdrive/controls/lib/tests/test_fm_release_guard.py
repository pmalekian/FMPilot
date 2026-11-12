"""FM Pilot Release Guard: limits the torque controller's proportional push against the driver steering into a turn,
Rivian only, off by default (Most Torque Against You 150 or 75 counts)."""
import math
from types import SimpleNamespace

import pytest

from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.fm_release_guard import (BASE_COUNTS, BLEND_S, MAX_DEFAULT, PARAM_ENABLED,
                                                                          PARAM_MAX, RELEASE_S, SPEEDS, FmReleaseGuard)

LAF = 2.75  # learned latAccelFactor in the Nov 11 logs
MPH = 0.44704
LEFT_TURN = -0.08  # 1/m, right-positive: a tight left turn
HANDS_INTO_LEFT = 2.0  # Nm, left-positive: pressing into a left turn
RIVIAN = SimpleNamespace(brand="rivian")


class FakeParams:
  def __init__(self, enabled=True, max_counts=150, **extra):
    self.values = {PARAM_ENABLED: enabled, PARAM_MAX: max_counts, **extra}

  def get_bool(self, key):
    return bool(self.values.get(key))

  def get(self, key, return_default=False):
    return self.values.get(key)


def pid(p, f=0.0, i=0.0, limit=1.5 * LAF):
  return SimpleNamespace(p=p, i=i, d=0.0, f=f, control=max(-limit, min(limit, p + i + f)), pos_limit=limit, neg_limit=-limit)


def limit_for(counts, fm_scale=1.0):
  return counts / BASE_COUNTS * LAF / fm_scale


def step(g, P, v=3.5, pressed=True, hand=HANDS_INTO_LEFT, k=LEFT_TURN, fm_scale=1.0):
  return g.update(P, True, v, pressed, hand, k, LAF, fm_scale)


def test_off_by_default_and_rivian_only():
  from openpilot.common.params import Params
  assert not Params().get_bool(PARAM_ENABLED)
  assert int(Params().get(PARAM_MAX, return_default=True)) == MAX_DEFAULT == 150
  g = FmReleaseGuard(RIVIAN, FakeParams(enabled=False))
  P = pid(10.0)
  assert step(g, P) == pytest.approx(P.control) and P.p == 10.0 and not g.active
  g = FmReleaseGuard(SimpleNamespace(brand="toyota"), FakeParams())
  P = pid(10.0)
  step(g, P)
  assert P.p == 10.0 and not g.available


@pytest.mark.parametrize("key", ["LateralJerkTorqueController", "NeuralNetworkLateralControl"])
def test_stands_down_with_controllers_that_replace_the_output(key):
  g = FmReleaseGuard(RIVIAN, FakeParams(**{key: True}))
  P = pid(10.0)
  step(g, P)
  assert not g.available and P.p == 10.0


@pytest.mark.parametrize("counts", [150, 75])
def test_limits_p_against_you_to_the_setting_and_keeps_feedforward(counts):
  g = FmReleaseGuard(RIVIAN, FakeParams(max_counts=counts))
  # left turn, hands left (into it), truck tighter than asked: P pushes right (+), feedforward left (-)
  P = pid(10.0, f=-0.6)
  out = step(g, P)
  assert g.active
  assert P.p == pytest.approx(limit_for(counts))
  assert P.f == -0.6
  assert out == pytest.approx(limit_for(counts) - 0.6) == P.control
  assert g.cut == pytest.approx(10.0 - limit_for(counts))


def test_counts_hold_in_higher_torque_full_mode():
  g = FmReleaseGuard(RIVIAN, FakeParams())
  P = pid(10.0)
  step(g, P, fm_scale=1.5)
  assert P.p * 1.5 == pytest.approx(limit_for(150))  # Full mode scales the output by 1.5 after the controller


def test_never_cuts_below_the_limit_or_a_push_with_you():
  g = FmReleaseGuard(RIVIAN, FakeParams())
  P = pid(limit_for(150) * 0.9)
  step(g, P)
  assert not g.active and P.p == pytest.approx(limit_for(150) * 0.9)
  P = pid(-8.0)  # P pushing left, the same way as the hands: openpilot helping
  step(g, P)
  assert not g.active and P.p == -8.0


def test_wheel_reaction_and_catching_an_overshoot_never_trigger_it():
  g = FmReleaseGuard(RIVIAN, FakeParams())
  # hands read right (against openpilot's left push) in a left turn: the loose wheel's reaction, or steering out
  P = pid(-10.0)
  step(g, P, hand=-1.5)
  assert not g.active and P.p == -10.0
  # truck not turning yet
  P = pid(10.0)
  step(g, P, k=-0.002)
  assert not g.active and P.p == 10.0
  # hands not pressed (under the 1 Nm threshold) from the start
  g = FmReleaseGuard(RIVIAN, FakeParams())
  P = pid(10.0)
  step(g, P, pressed=False, hand=0.6)
  assert not g.active


def test_right_turn_mirrors():
  g = FmReleaseGuard(RIVIAN, FakeParams())
  P = pid(-10.0)  # right turn, hands right (-), P pushes left (-)
  step(g, P, hand=-2.0, k=0.08)
  assert g.active and P.p == pytest.approx(-limit_for(150))


def test_speed_fade():
  mid = (SPEEDS[0] + SPEEDS[1]) / 2
  g = FmReleaseGuard(RIVIAN, FakeParams())
  P = pid(10.0)
  step(g, P, v=mid)
  assert P.p == pytest.approx(10.0 - 0.5 * (10.0 - limit_for(150)))
  P = pid(10.0)
  step(g, P, v=SPEEDS[1] + 0.1)
  assert not g.active and P.p == 10.0
  assert SPEEDS[0] == pytest.approx(15 * MPH) and SPEEDS[1] == pytest.approx(20 * MPH)


def test_release_fades_and_ends_when_p_flips():
  g = FmReleaseGuard(RIVIAN, FakeParams())
  for _ in range(10):
    step(g, pid(10.0))
  # let go: still cut, fading out
  P = pid(10.0)
  step(g, P, pressed=False, hand=0.0, k=-0.03)
  assert g.active and limit_for(150) < P.p < 10.0
  frames = 1
  while g.amount > 0.0:
    step(g, pid(10.0), pressed=False, hand=0.0, k=-0.03)
    frames += 1
  assert frames == pytest.approx(RELEASE_S / DT_CTRL, abs=2)
  # P flips (wheel past openpilot's target): never touched, and the guard is over
  g = FmReleaseGuard(RIVIAN, FakeParams())
  step(g, pid(10.0))
  P = pid(-6.0)
  step(g, P, pressed=False, hand=0.0, k=0.01)
  assert not g.active and P.p == -6.0 and g.amount == 0.0
  P = pid(10.0)
  step(g, P, pressed=False, hand=0.0, k=-0.01)
  assert not g.active and P.p == 10.0


def test_hands_reversing_ends_it_at_once():
  g = FmReleaseGuard(RIVIAN, FakeParams())
  step(g, pid(10.0))
  P = pid(10.0)
  step(g, P, hand=-1.5)  # now steering out of the left turn
  assert not g.active and P.p == 10.0 and g.amount == 0.0


def test_setting_blends_and_idle_resets():
  params = FakeParams(enabled=False)
  g = FmReleaseGuard(RIVIAN, params)
  params.values[PARAM_ENABLED] = True  # read on the next frame (then once a second)
  P = pid(10.0)
  step(g, P)
  assert 0.0 < g.enabled < 1.0 and P.p > limit_for(150)
  for _ in range(int(BLEND_S / DT_CTRL) + 2):
    step(g, pid(10.0))
  assert g.enabled == 1.0
  g.idle()
  assert g.amount == 0.0 and not g.active
  P = pid(10.0)
  step(g, P, pressed=False, hand=0.0)
  assert P.p == 10.0


def test_bad_setting_falls_back_to_150():
  g = FmReleaseGuard(RIVIAN, FakeParams(max_counts=999))
  assert g.max_counts == 150
  g = FmReleaseGuard(RIVIAN, FakeParams(max_counts=float("nan")))
  assert g.max_counts == 150


# --- in the controllers ---

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


def _vm(CP):
  from opendbc.car.vehicle_model import VehicleModel
  return VehicleModel(CP)


def _output(version, enabled, pressed=True, hand=HANDS_INTO_LEFT, frames=40):
  """Left turn at 8 mph, the wheel held at 340 deg while openpilot asks for 30% less (Rebel Legion, 00000067--0)."""
  from opendbc.car.structs import car
  from openpilot.sunnypilot.selfdrive.controls.lib.fm_low_speed_gain import FmLowSpeedGain
  from openpilot.sunnypilot.selfdrive.controls.lib.fm_low_speed_ff import FmLowSpeedFF
  CP, CP_SP, CI = _rivian()
  VM = _vm(CP)
  lac = _controllers()[version](CP, CP_SP, CI, DT_CTRL)
  lac.fm_low_speed = FmLowSpeedGain(SimpleNamespace(brand="toyota"))
  lac.fm_low_speed_ff = FmLowSpeedFF(SimpleNamespace(brand="toyota"))
  lac.fm_release_guard = FmReleaseGuard(RIVIAN, FakeParams(enabled=enabled))
  v, wheel = 8 * MPH, 340.0
  CS = car.CarState(vEgo=v, steeringAngleDeg=wheel, steeringTorque=hand, steeringPressed=pressed)
  lp = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  k = -VM.calc_curvature(math.radians(wheel), v, 0.0) * 0.7
  out, log = 0.0, None
  for _ in range(frames):
    out, _, log = lac.update(True, CS, VM, lp, False, k, None, False, 0.2)
  return out, log, lac


@pytest.mark.parametrize("version", ["v0", "v1"])
def test_controllers_limit_the_push_against_you(version):
  off, off_log, _ = _output(version, False)
  on, on_log, lac = _output(version, True)
  # output is left-positive; openpilot pushes right (negative) against the hands
  assert off < 0.0 and on < 0.0
  assert abs(on) < abs(off)
  assert lac.fm_release_guard.active
  assert on_log.p == pytest.approx(lac.fm_release_guard.limit)  # the log shows the P actually used
  assert on_log.f == pytest.approx(off_log.f)  # feedforward untouched
  # with the hands not pressing into the turn, identical
  a, _, _ = _output(version, False, pressed=False, hand=0.3)
  b, _, _ = _output(version, True, pressed=False, hand=0.3)
  assert a == pytest.approx(b)


@pytest.mark.parametrize("enforce,tune", [(False, 0.0), (True, 0.0), (True, 1.0)])
def test_every_selectable_torque_controller_has_the_hook(enforce, tune):
  from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
  CP, CP_SP, CI = _rivian()
  vals = {"EnforceTorqueControl": enforce, "TorqueControlTune": tune}
  host = SimpleNamespace(CP=CP, CP_SP=CP_SP,
                         params=SimpleNamespace(get_bool=lambda k: bool(vals[k]), get=lambda k, **kw: vals[k]))
  chosen = ControlsExt.initialize_lateral_control(host, _controllers()["v1"](CP, CP_SP, CI, DT_CTRL), CI, DT_CTRL)
  assert isinstance(getattr(chosen, "fm_release_guard", None), FmReleaseGuard), type(chosen).__module__
  assert chosen.fm_release_guard.available
