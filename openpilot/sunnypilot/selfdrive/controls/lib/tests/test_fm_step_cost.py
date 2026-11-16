"""FM Pilot: the hooks that run on every 10 ms control step do almost nothing when there is nothing to do, and what they
compute and log is unchanged.

Before: with no turn, Lane Guard off and every blend settled, FM Pilot's hooks in controlsd still made about a dozen
numpy scalar calls, ran Lane Guard's update, wrote 28 log fields and updated a second SubMaster on every step: about
150 us a step on a PC (68 of it the SubMaster). After: about 20.
"""
import math
import random
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import custom, messaging
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.lane_guard import LANE_CHANGE_COOLDOWN
from openpilot.sunnypilot.selfdrive.controls.lib import fm_low_speed_gain, fm_torque_mode
from openpilot.sunnypilot.selfdrive.controls.lib.fm_lateral import FmLateral
from openpilot.sunnypilot.selfdrive.controls.lib.fm_low_speed_gain import FmLowSpeedGain
from openpilot.sunnypilot.selfdrive.controls.lib.fm_math import step_toward
from openpilot.sunnypilot.selfdrive.controls.lib.fm_torque_mode import FULL, HEADROOM, OFF, FmTorqueMode, high_torque_ratio
from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_assist import (PARAM_BOOST, PARAM_CARRY, PARAM_HOLD, PARAM_HOLD_DRIVER,
                                                                        PARAM_HOLD_FOLLOW_EXIT, PARAM_HOLD_PATH, PARAM_LEAD,
                                                                        PARAM_PHASE_IN, SOURCE_NONE, FmTurnAssist, TurnDirection)

LEFT, RIGHT, NONE = TurnDirection.turnLeft, TurnDirection.turnRight, TurnDirection.none


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key))

  def get(self, key, return_default=False, **kwargs):
    return self.values.get(key)


def test_step_toward_is_np_clip():
  rng = random.Random(1)
  for _ in range(20000):
    value, target = rng.choice([0.0, 1.0, rng.uniform(-2, 2)]), rng.choice([0.0, 1.0, rng.uniform(-2, 2)])
    step = rng.choice([DT_CTRL, DT_CTRL / 0.5, rng.uniform(0, 1)])
    assert step_toward(value, target, step) == float(np.clip(target, value - step, value + step))
  assert step_toward(0.3, 0.3, 0.01) == 0.3
  assert math.isnan(step_toward(0.3, float("nan"), 0.01))  # a NaN target passes through, as np.clip's does


# ---- Turn Assist's log record ----

def _fill_every_field(a: FmTurnAssist, msg) -> None:
  """fill_log as it was: every field written on every step. The reference."""
  msg.sign = int(a.sign if (a.log_source != SOURCE_NONE or a.log_exit > 0.0) else 0)
  msg.source = int(a.log_source)
  msg.amount = float(a.amount)
  msg.requestIn = float(a.log_in)
  msg.requestOut = float(a.log_out)
  msg.boost = float(a.log_boost)
  msg.lead = float(a.log_lead)
  msg.driverAdds = float(a.log_driver)
  msg.holdPath = float(a.log_path)
  msg.holdFloor = float(a.log_floor)
  msg.nudge = float(a.log_nudge)
  msg.smoothCut = float(a.log_smooth_cut)
  msg.exitFade = float(a.log_exit)
  msg.holdActive = bool(a.hold_enabled and a.sign != 0 and not a.hold_released and a.log_source != SOURCE_NONE)
  msg.holdPeak = float(a.peak)
  msg.holdFloorLevel = float(a.log_floor_level)
  msg.turned = float(math.degrees(a.turned))
  msg.holdTurned = float(math.degrees(a.heading))
  msg.holdTime = float(a.turn_time)
  msg.releaseReason = int(a.release_reason)
  msg.earlyCorrection = bool(a.log_early)
  msg.againstTime = float(a.against_time)


def _path(curvature):
  xs = [float(x) for x in np.linspace(0, 40, 33)]
  return SimpleNamespace(position=SimpleNamespace(x=xs, y=[0.5 * curvature * x * x for x in xs]))


def test_turn_assist_log_is_the_same_record_with_fewer_writes():
  rng = random.Random(2)
  every = {PARAM_BOOST: True, PARAM_HOLD: True, PARAM_LEAD: True, PARAM_HOLD_PATH: True, PARAM_HOLD_DRIVER: True,
           PARAM_CARRY: True, PARAM_PHASE_IN: True, PARAM_HOLD_FOLLOW_EXIT: True, "FM_TurnLeadMinSpeed": 2.0}
  a = FmTurnAssist(FakeParams(**every))
  direction, pressed, torque, lat, v, actual = NONE, False, 0.0, True, 5.0, 0.0
  seen_idle = seen_turn = 0
  for i in range(30000):
    if rng.random() < 0.004:
      direction = rng.choice([NONE, LEFT, RIGHT])
    if rng.random() < 0.01:
      pressed, torque = not pressed, rng.uniform(-3, 3)
    if rng.random() < 0.002:
      lat = not lat
    v = min(max(v + rng.gauss(0, 0.03), 0.0), 12.0)
    request = 0.05 * math.sin(i / 70.0) + rng.gauss(0, 0.004)
    actual += (request - actual) * 0.05
    a.update(request, direction, v, actual, pressed, lat, model_v2=_path(rng.gauss(0, 0.05)), a_ego=rng.gauss(0, 0.5),
             steering_torque=torque)
    new, ref = custom.CarControlSP.new_message(), custom.CarControlSP.new_message()
    a.fill_log(new.turnAssist)
    _fill_every_field(a, ref.turnAssist)
    assert new.to_dict() == ref.to_dict(), i
    seen_idle += new.turnAssist.sign == 0
    seen_turn += new.turnAssist.sign != 0
  assert seen_idle > 3000 and seen_turn > 3000  # both kinds of step were covered


def test_no_turn_writes_only_the_request():
  class Recorder:
    def __init__(self):
      object.__setattr__(self, "written", [])

    def __setattr__(self, name, value):
      self.written.append(name)

  a = FmTurnAssist(FakeParams(**{PARAM_HOLD: True, PARAM_LEAD: True}))
  a.update(0.01, NONE, 30.0, 0.01, False, True)
  msg = Recorder()
  a.fill_log(msg)
  assert msg.written == ["requestIn", "requestOut"]
  a.update(0.0, NONE, 30.0, 0.0, False, True)
  msg = Recorder()
  a.fill_log(msg)
  assert msg.written == []  # driving straight: the message's defaults already say it all


# ---- Lane Guard is only run while it has something to do ----

def _lateral(mode):
  params = FakeParams(FM_LaneKeepMode=mode)
  lat = FmLateral(SimpleNamespace(brand="rivian", wheelbase=3.08), params)
  calls = []
  real = lat.lane_guard.update

  def update(v_ego, model_v2, active, blinker=False, left_hand_traffic=False):
    calls.append(active)
    return real(v_ego, model_v2, active, blinker, left_hand_traffic)
  lat.lane_guard.update = update
  return lat, params, calls


def _model():
  # a lane 3.6 m wide with the planned path drifting to within reach of the right line: Lane Guard pushes left
  xs = [float(x) for x in np.linspace(0, 100, 33)]
  ts = [float(t) for t in np.linspace(0, 10, 33)]

  def line(y):
    return SimpleNamespace(x=xs, y=[y] * 33)
  return SimpleNamespace(meta=SimpleNamespace(laneChangeState=0), position=SimpleNamespace(x=xs, y=[1.2] * 33, t=ts),
                         laneLines=[line(-5.4), line(-1.8), line(1.8), line(5.4)], laneLineProbs=[0.9] * 4, laneLineStds=[0.1] * 4,
                         roadEdges=[line(-6.0), line(6.0)], roadEdgeStds=[0.2, 0.2])


CS = SimpleNamespace(vEgo=30.0, aEgo=0.0, steeringPressed=False, steeringTorque=0.0, leftBlinker=False, rightBlinker=False)


@pytest.mark.parametrize("mode", [0, 1, 3])
def test_lane_guard_is_not_run_while_another_mode_is_selected(mode):
  lat, _, calls = _lateral(mode)
  for _ in range(300):
    assert lat.update(0.002, _model(), CS, True, mode == 1, 0.002) == 0.002
  assert calls == [] and lat.lane_guard_resting


def test_lane_guard_runs_while_selected_and_fades_out_after_it_is_switched_off():
  lat, params, calls = _lateral(2)
  model = _model()
  for _ in range(300):
    out = lat.update(0.0, model, CS, True, False, 0.0)
  assert len(calls) == 300 and all(calls) and out < 0.0  # pushing left, away from the right line
  params.values["FM_LaneKeepMode"] = 0
  lat.read_params()
  outs = [lat.update(0.0, model, CS, True, False, 0.0) for _ in range(2000)]
  faded = next(i for i, o in enumerate(outs) if o == 0.0)
  assert 10 < faded < 1500 and outs[0] < 0.0  # it lets go at its own rate, not in one step
  assert all(o == 0.0 for o in outs[faded:])
  assert len(calls) == 300 + faded + 1 and not any(calls[300:])  # then it is left alone
  assert lat.lane_guard_resting


def test_lane_guard_switched_on_while_driving_starts_as_after_a_lane_change():
  lat, params, calls = _lateral(0)
  model = _model()
  for _ in range(50):
    lat.update(0.0, model, CS, True, False, 0.0)
  params.values["FM_LaneKeepMode"] = 2
  lat.read_params()
  first = lat.update(0.0, model, CS, True, False, 0.0)
  assert first == 0.0 and len(calls) == 1 and not lat.lane_guard_resting
  assert lat.lane_guard.lane_change_timer == pytest.approx(LANE_CHANGE_COOLDOWN - DT_CTRL)
  outs = [lat.update(0.0, model, CS, True, False, 0.0) for _ in range(int(3.0 / DT_CTRL))]
  assert all(o == 0.0 for o in outs[:int(LANE_CHANGE_COOLDOWN / DT_CTRL) - 2])  # lines unguarded for the first second
  assert outs[-1] < 0.0  # then it guards the line


# ---- Higher Torque and Calmer Low-Speed Correction ----

def _torque_mode(mode, cap_level=8):
  from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP
  CP_SP = SimpleNamespace(flags=int(RivianFlagsSP.HIGH_TORQUE_PANDA), safetyParam=2 | (cap_level << 2))
  return FmTorqueMode(SimpleNamespace(brand="rivian"), CP_SP, FakeParams(FM_TorqueMode=mode))


@pytest.mark.parametrize("mode", [OFF, HEADROOM, FULL])
def test_torque_mode_values_are_unchanged(mode):
  tm = _torque_mode(mode)
  w_h = w_f = 0.0
  step = DT_CTRL / fm_torque_mode.BLEND_S
  rng = random.Random(3)
  for _ in range(400):
    v = rng.uniform(0.0, 35.0)
    w_h = float(np.clip(1.0 if mode == HEADROOM else 0.0, w_h - step, w_h + step))
    w_f = float(np.clip(1.0 if mode == FULL else 0.0, w_f - step, w_f + step))
    extra = high_torque_ratio(v, tm.cap) - 1.0
    assert tm.update(v) == (1.0 + extra * w_h, 1.0 + extra * w_f)


def test_torque_mode_builds_its_speed_curves_once(monkeypatch):
  tm = _torque_mode(HEADROOM)

  def no_lookups(cap):
    raise AssertionError("the speed curves are looked up once per drive, not on every control step")
  monkeypatch.setattr(fm_torque_mode, "_lookups", no_lookups)
  for _ in range(int(fm_torque_mode.BLEND_S / DT_CTRL) + 1):  # the mode blends in over a second
    for v in (0.0, 5.0, 12.0, 30.0):
      tm.update(v)
  assert tm.update(3.0) == (pytest.approx(580 / 385), 1.0)


def test_low_speed_gain_factor_is_the_interpolation():
  rng = random.Random(4)
  gain = FmLowSpeedGain(SimpleNamespace(brand="rivian"), FakeParams(FM_LowSpeedGainEnabled=True, FM_LowSpeedGainPercent=70))
  for scale in (0.4, 0.55, 0.7, 0.999, 1.0):
    gain.scale = scale
    for _ in range(500):
      v = rng.choice([0.0, 6.99, 7.0, 8.0, 9.0, 9.01, 40.0 * rng.random()])
      assert gain.factor(v) == float(np.interp(v, fm_low_speed_gain.SPEEDS, [scale, 1.0]))


# ---- what controlsd logs ----

def _host(guard=None, low_ff=None, low_gain=None, turn_assist=None):
  lc = SimpleNamespace(active=False, holding=False, correction=0.0, center_error=0.0, clipped=False, single_line=False, line_side=0)
  lac = SimpleNamespace(**{k: v for k, v in (("fm_release_guard", guard), ("fm_low_speed_ff", low_ff), ("fm_low_speed", low_gain))
                           if v is not None})
  return SimpleNamespace(LaC=lac, lane_centering=lc, get_lead_data=lambda *a: None,
                         fm_lateral=SimpleNamespace(turn_assist=turn_assist))


def _sm(v_ego=30.0):
  cs = messaging.new_message('carState')
  cs.carState.vEgo = v_ego
  return {'radarState': messaging.new_message('radarState').radarState,
          'selfdriveStateSP': messaging.new_message('selfdriveStateSP').selfdriveStateSP, 'carState': cs.carState}


def test_controlsd_logs_the_same_values():
  from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
  guard = SimpleNamespace(active=True, amount=0.8, enabled=0.5, cut=-1.25, limit=1.07)
  low_gain = FmLowSpeedGain(SimpleNamespace(brand="rivian"), FakeParams(FM_LowSpeedGainEnabled=True, FM_LowSpeedGainPercent=70))
  cc = ControlsExt.state_control_ext(_host(guard, SimpleNamespace(last=0.31), low_gain), _sm(3.0))
  assert (cc.releaseGuard.active, cc.releaseGuard.amount, cc.releaseGuard.cut, cc.releaseGuard.limit) == \
         (True, pytest.approx(0.4), pytest.approx(-1.25), pytest.approx(1.07))
  assert cc.fmLowSpeed.feedforward == pytest.approx(0.31) and cc.fmLowSpeed.gainScale == pytest.approx(0.7)
  # at rest: the guard's limit is still logged (as before), nothing else is written, and the gain reads 1 at speed
  guard = SimpleNamespace(active=False, amount=0.0, enabled=1.0, cut=0.0, limit=1.07)
  cc = ControlsExt.state_control_ext(_host(guard, SimpleNamespace(last=0.0), low_gain), _sm(30.0))
  assert (cc.releaseGuard.active, cc.releaseGuard.amount, cc.releaseGuard.cut) == (False, 0.0, 0.0)
  assert cc.releaseGuard.limit == pytest.approx(1.07)
  assert cc.fmLowSpeed.feedforward == 0.0 and cc.fmLowSpeed.gainScale == 1.0
  # a guard that never ran leaves the struct unset
  guard = SimpleNamespace(active=False, amount=0.0, enabled=0.0, cut=0.0, limit=0.0)
  cc = ControlsExt.state_control_ext(_host(guard), _sm())
  assert not cc._has("releaseGuard") and cc.fmLowSpeed.gainScale == 1.0


def test_speed_fade_is_only_worked_out_during_a_turn(monkeypatch):
  from openpilot.sunnypilot.selfdrive.controls.lib import fm_turn_assist
  a = FmTurnAssist(FakeParams(**{PARAM_BOOST: True, "FM_TurnBoostRight": 1.2}))
  calls = []
  real = np.interp

  def interp(*args, **kwargs):
    calls.append(args[0])
    return real(*args, **kwargs)
  monkeypatch.setattr(fm_turn_assist.np, "interp", interp)
  for _ in range(100):
    assert a.update(0.01, NONE, 30.0, 0.01, False, True) == 0.01
  assert calls == []
  out = 0.0
  for _ in range(100):
    out = a.update(0.01, RIGHT, 10 * CV.MPH_TO_MS, 0.01, False, True)
  assert out == pytest.approx(0.012) and len(calls) == 100
