"""FM Pilot: the Accel Boost picker (fm_accel_boost_mode.py).

Ported from fm-dev-nav-settle ca08aac9 (selfdrive/controls/lib/tests/test_fm_boost_mode.py: Predictive is Sunny Boost now,
Set Speed is EVP Set Speed, the default is Off), plus tests for modes 4 and 5, what every mode adds, the picker deciding
alone over sunnypilot's AccelBoostSP and AP's AccelerationControl toggles, and Off adding nothing. Mode 1 (Gas, Gas Override
Boost) is removed: a stored 1 reads as Off.
"""
import random
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from openpilot.cereal import custom, log
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.accel_boost import AccelBoostSP, ACCEL_BOOST_MAX as SUNNY_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.acceleration_control import AccelerationControl, ACCELERATION_CONTROL_BOOST
from openpilot.sunnypilot.selfdrive.controls.lib.fm_accel_boost_mode import (
  DEFAULT_MODE, MODES, MODE_AP_ACCEL_CONTROL, MODE_EVP_SET_SPEED, MODE_OFF, MODE_OP_PERSONALITY, MODE_SUNNY, PARAM_MODE,
  PARAM_CURVE_HOLD, READ_FRAMES, REMOVED_MODES, FmAccelBoostMode, OP_BAR_MODEL_ACCEL, boost_max, parse_mode,
)
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost as OpAccelBoost, get_accel_boost
from openpilot.selfdrive.controls.lib.curve_speed import CurveSpeed
from openpilot.sunnypilot.selfdrive.controls.lib.set_speed_assist import MAX_BOOST as EVP_MAX

P = log.LongitudinalPersonality
CP = structs.CarParams(openpilotLongitudinalControl=True)


class Personality(int):
  """Like the capnp enum in selfdriveState: compares as its number and has .raw (AP's Acceleration Control reads it)."""
  @property
  def raw(self):
    return int(self)


class FakeParams:
  def __init__(self, mode=None):
    self.values = {} if mode is None else {PARAM_MODE: mode}

  def get(self, key, return_default=False):
    default = {PARAM_MODE: DEFAULT_MODE, PARAM_CURVE_HOLD: True}[key] if return_default else None
    return self.values.get(key, default)


def make_sm(enabled=True, gas_pressed=False, mph=30, brake=0.0, gas=0.0, should_stop=False, curvature=0.0,
            personality=P.aggressive, probs=6, a_ego=0.0, lead=False, desired_accel=0.5, plan_ahead=2.0, plan_lat_accel=0.0,
            roll=0.0):
  # plan_ahead: m/s the model's 10 s plan averages above the current speed (OP Personality's gate: full from 1.0)
  # plan_lat_accel: lateral accel (m/s^2) the plan reaches 2 s ahead (openpilot's CurveSpeed: a curve from 2.8)
  v = mph * CV.MPH_TO_MS
  plan_v = max(v, 1.0) + plan_ahead
  position = SimpleNamespace(x=[0.0, plan_v * 2.0, plan_v * 10.0], t=[0.0, 2.0, 10.0])
  velocity = SimpleNamespace(x=[v, v, v])
  orientation_rate = SimpleNamespace(z=[0.0, plan_lat_accel / max(v, 0.1), 0.0])
  meta = SimpleNamespace(disengagePredictions=SimpleNamespace(brakePressProbs=[brake] * probs, gasPressProbs=[gas] * probs))
  return {
    'selfdriveState': SimpleNamespace(enabled=enabled, personality=Personality(personality)),
    'carState': SimpleNamespace(gasPressed=gas_pressed, vEgo=mph * CV.MPH_TO_MS, aEgo=a_ego),
    'vehicleParameters': SimpleNamespace(roll=roll),
    'modelV2': SimpleNamespace(meta=meta, position=position, velocity=velocity, orientationRate=orientation_rate,
                               action=SimpleNamespace(shouldStop=should_stop,
                                                                                    desiredCurvature=curvature,
                                                                                    desiredAcceleration=desired_accel)),
    'radarState': SimpleNamespace(leadOne=SimpleNamespace(present=lead)),
  }


def run(boost, frames, e2e=0.5, **kwargs):
  out = None
  for _ in range(frames):
    # model wants 0.5 m/s^2, classic planner and cruise allow 1.5: the model is limiting
    out = boost.update(make_sm(desired_accel=e2e, **kwargs), e2e, 1.5, 1.5)
  return out


def picker(mode):
  return FmAccelBoostMode(CP, FakeParams(mode))


class TestParseMode:
  @pytest.mark.parametrize("value,mode", [(0, MODE_OFF), (1, MODE_OFF), (2, MODE_SUNNY), ("2", MODE_SUNNY),
                                          (3, MODE_EVP_SET_SPEED), ("3", MODE_EVP_SET_SPEED), (4, MODE_AP_ACCEL_CONTROL),
                                          (5, MODE_OP_PERSONALITY), (None, MODE_OFF), (7, MODE_OFF), (-1, MODE_OFF),
                                          ("x", MODE_OFF)])
  def test_parse(self, value, mode):
    assert parse_mode(value) == mode

  def test_values_match_the_log_enum(self):
    enum = custom.FmPlan.AccelBoostMode.schema.enumerants  # fmPlan.accelBoostMode
    assert enum == {'off': MODE_OFF, 'gas': 1, 'sunnyBoost': MODE_SUNNY, 'evpSetSpeed': MODE_EVP_SET_SPEED,
                    'apAccelControl': MODE_AP_ACCEL_CONTROL, 'opPersonality': MODE_OP_PERSONALITY}
    assert REMOVED_MODES == (1,) and 1 not in MODES  # gas @1 stays in the log enum but is never sent

  def test_default_is_off(self):
    assert DEFAULT_MODE == MODE_OFF
    assert FmAccelBoostMode(CP, FakeParams()).mode == MODE_OFF


class TestSunnyBoost:
  """Mode 2 runs sunnypilot's AccelBoostSP.update() unchanged (ca08aac9's Predictive tests, on upstream's code)."""
  def test_no_reason_to_slow_builds_to_personality_max(self):
    for personality, cap in zip((P.aggressive, P.standard, P.relaxed), SUNNY_MAX, strict=True):
      boost = picker(MODE_SUNNY)
      assert run(boost, 400, personality=personality) == pytest.approx(0.5 + cap)
      assert boost.applied == pytest.approx(cap)

  def test_model_wants_to_brake_no_boost(self):
    assert run(picker(MODE_SUNNY), 400, brake=0.5) == 0.5

  def test_strong_gas_prediction_overrides_brake_prediction(self):
    assert run(picker(MODE_SUNNY), 400, brake=0.5, gas=0.9) > 0.5

  def test_should_stop_no_boost(self):
    assert run(picker(MODE_SUNNY), 400, should_stop=True) == 0.5

  def test_fades_in_tight_curves(self):
    v = 30 * CV.MPH_TO_MS
    assert run(picker(MODE_SUNNY), 400, curvature=2.5 / v ** 2) == 0.5

  def test_no_boost_when_model_brakes(self):
    boost = picker(MODE_SUNNY)
    run(boost, 400)
    assert run(boost, 1, e2e=-0.8) == -0.8

  def test_disengaged_resets(self):
    boost = picker(MODE_SUNNY)
    run(boost, 400)
    assert run(boost, 1, enabled=False) == 0.5
    assert boost.sunny.boost == 0.0

  def test_same_as_upstream_class(self):
    boost, upstream = picker(MODE_SUNNY), AccelBoostSP()
    for frame in range(300):
      sm = make_sm(personality=P.standard, brake=0.05 if frame < 150 else 0.25)
      assert boost.update(sm, 0.4, 1.5, 1.5) == upstream.update(sm, 0.4, 1.5, 1.5)


class TestEachModesAmount:
  """What each mode adds on constructed inputs, and that longitudinalPlan.accelBoost (applied) is exactly that."""
  def test_off(self):
    boost = picker(MODE_OFF)
    assert run(boost, 10) == 0.5
    assert run(boost, 2000, gas_pressed=True, gas=0.9, mph=40) == 0.5
    assert boost.applied == 0.0

  def test_removed_gas_mode_is_off(self):
    boost = picker(1)  # a stored 1 (the removed Gas mode)
    assert boost.mode == MODE_OFF
    run(boost, 10)
    for _ in range(4):  # the presses that used to fill Gas Override Boost add nothing now
      assert run(boost, 2000, gas_pressed=True) == 0.5
      run(boost, 10)
    assert boost.applied == 0.0

  @pytest.mark.parametrize("personality,idx", [(P.aggressive, 0), (P.standard, 1), (P.relaxed, 2)])
  def test_evp_set_speed_at_highway_speed(self, personality, idx):
    boost = picker(MODE_EVP_SET_SPEED)
    assert run(boost, 400, mph=40, brake=0.5, personality=personality) == pytest.approx(0.5 + EVP_MAX[idx])
    assert boost.applied == pytest.approx(EVP_MAX[idx])

  def test_evp_set_speed_waits_above_20_mph(self):
    assert run(picker(MODE_EVP_SET_SPEED), 400, mph=15) == 0.5

  @pytest.mark.parametrize("personality", [P.aggressive, P.standard, P.relaxed])
  def test_ap_accel_control(self, personality):
    boost = picker(MODE_AP_ACCEL_CONTROL)
    # AP's numbers: full lift from a 0.1 m/s^2 model request and 20 mph; 0.3 / 0.2 / 0.1 by personality
    assert run(boost, 1, e2e=0.1, mph=25, personality=personality) == pytest.approx(0.1 + ACCELERATION_CONTROL_BOOST[personality])
    assert boost.applied == pytest.approx(ACCELERATION_CONTROL_BOOST[personality])
    assert boost.acceleration_control_boost == pytest.approx(ACCELERATION_CONTROL_BOOST[personality])
    assert run(boost, 1, e2e=-0.2, mph=25, personality=personality) == -0.2  # never on a braking request
    assert run(boost, 1, e2e=0.1, mph=8, personality=personality) == 0.1  # none below 10 mph

  def test_ap_accel_control_follows_the_curve_throttle_interlock(self):
    boost = picker(MODE_AP_ACCEL_CONTROL)
    sm = make_sm(desired_accel=0.1, mph=25, personality=P.standard)
    assert boost.update(sm, 0.1, 1.5, 1.5, throttle_scale=0.5) == pytest.approx(0.2)

  @pytest.mark.parametrize("personality", [P.aggressive, P.standard, P.relaxed])
  def test_op_personality(self, personality):
    boost = picker(MODE_OP_PERSONALITY)
    bias, gain = get_accel_boost(personality)
    # openpilot #39248: bias (back over ~1 s) plus gain x the model's request, on a clear road
    assert run(boost, 400, mph=30, personality=personality) == pytest.approx(0.5 + bias + gain * 0.5, abs=1e-4)
    assert boost.applied == pytest.approx(bias + gain * 0.5, abs=1e-4)

  def test_op_personality_is_openpilots_class(self):
    boost, upstream = picker(MODE_OP_PERSONALITY), OpAccelBoost()
    rng = random.Random(0)
    for _ in range(600):
      sm = make_sm(mph=rng.uniform(0, 60), plan_ahead=rng.uniform(-2, 3), personality=rng.choice((P.aggressive, P.standard, P.relaxed)))
      e2e = rng.uniform(-1.5, 1.5)
      assert boost.update(sm, e2e, 1.5, 1.5) == upstream.update(sm, e2e, 0.0)

  def test_op_personality_holds_off_when_the_plan_slows(self):
    boost = picker(MODE_OP_PERSONALITY)
    run(boost, 400, personality=P.aggressive)
    assert boost.applied > 0.3
    assert run(boost, 30, personality=P.aggressive, plan_ahead=-1.0) == pytest.approx(0.5)  # bias drops at once, no gain

  def test_op_personality_holds_off_in_curves_by_default(self):
    boost = picker(MODE_OP_PERSONALITY)  # FM_OpBoostCurveHold unset: on
    assert boost.hold_in_curves
    run(boost, 400, personality=P.aggressive)
    assert boost.applied > 0.3
    # a 3 m/s^2 curve 2 s ahead: openpilot's CurveSpeed slows for it, so the boost stops at once (bias and gain)
    assert run(boost, 1, personality=P.aggressive, plan_lat_accel=3.0) == 0.5
    assert boost.op_personality.curve_offset < 0.0
    assert run(boost, 100, personality=P.aggressive, plan_lat_accel=3.0) == 0.5
    # past the curve the offset ramps back to 0 (0.5 m/s^3), then the boost returns over about 1 s
    run(boost, 60, personality=P.aggressive)
    assert boost.op_personality.curve_offset == 0.0
    assert run(boost, 200, personality=P.aggressive) > 0.8

  def test_op_personality_curves_below_openpilots_limit_keep_the_boost(self):
    boost = picker(MODE_OP_PERSONALITY)
    assert run(boost, 400, personality=P.aggressive, plan_lat_accel=2.7) > 0.8  # under CurveSpeed's 2.8 m/s^2

  def test_op_personality_roll_counts_as_openpilot(self):
    boost = picker(MODE_OP_PERSONALITY)
    # 2.0 m/s^2 from the turn plus the road's bank (lat accel = v * yaw - g * roll): over 2.8 with roll -0.1
    run(boost, 400, personality=P.aggressive)
    assert run(boost, 1, personality=P.aggressive, plan_lat_accel=2.0, roll=-0.1) == 0.5

  def test_op_personality_curve_hold_off_boosts_in_curves(self):
    params = FakeParams(MODE_OP_PERSONALITY)
    params.values[PARAM_CURVE_HOLD] = False
    boost = FmAccelBoostMode(CP, params)
    assert not boost.hold_in_curves
    assert run(boost, 400, personality=P.aggressive, plan_lat_accel=3.0) > 0.8
    assert boost.op_personality.curve_offset < 0.0  # openpilot would be slowing; the boost runs anyway

  def test_curve_hold_is_read_every_params_period(self):
    params = FakeParams(MODE_OP_PERSONALITY)
    boost = FmAccelBoostMode(CP, params)
    params.values[PARAM_CURVE_HOLD] = False
    run(boost, READ_FRAMES)
    assert not boost.hold_in_curves

  @pytest.mark.parametrize("seed", range(3))
  def test_op_personality_is_openpilots_planner_step(self, seed):
    """Same numbers as openpilot's planner: CurveSpeed on the plan and roll, then AccelBoost with that offset."""
    boost, op_boost, op_curve = picker(MODE_OP_PERSONALITY), OpAccelBoost(), CurveSpeed(DT_MDL)
    rng = random.Random(seed)
    lat = 0.0
    for frame in range(1500):
      if frame % 40 == 0:
        lat = rng.choice((0.0, 1.0, 2.5, 3.2, -3.5))
      sm = make_sm(mph=rng.uniform(5, 60), plan_ahead=rng.uniform(-2, 3), plan_lat_accel=lat, roll=rng.uniform(-0.05, 0.05),
                   personality=rng.choice((P.aggressive, P.standard, P.relaxed)))
      e2e = rng.uniform(-1.5, 1.5)
      expected = op_boost.update(sm, e2e, op_curve.update(sm['modelV2'], sm['vehicleParameters'].roll))
      assert boost.update(sm, e2e, 1.5, 1.5) == expected

  def test_planner_reset_resets_op_personality(self):
    boost = picker(MODE_OP_PERSONALITY)
    run(boost, 400, personality=P.aggressive, plan_lat_accel=3.0)
    boost.planner_reset()
    assert boost.op_personality.op.bias_filter.x == 0.0 and boost.op_personality.curve_offset == 0.0
    assert boost.op_personality.curve_speed.accel_offset == 0.0

  def test_op_personality_plan_without_times_adds_nothing(self):
    boost = picker(MODE_OP_PERSONALITY)
    run(boost, 400, personality=P.aggressive)
    sm = make_sm(personality=P.aggressive)
    sm['modelV2'].position = SimpleNamespace(x=[0.0, 300.0], t=[])  # e.g. the longitudinal maneuver plant's model
    assert boost.update(sm, 0.5, 1.5, 1.5) == 0.5
    assert boost.op_personality.op.bias_filter.x == 0.0  # starts over

  def test_op_personality_unknown_personality_adds_nothing(self):
    assert run(picker(MODE_OP_PERSONALITY), 50, personality=9) == 0.5

  @pytest.mark.parametrize("mode", MODES)
  def test_applied_is_what_was_added(self, mode):
    boost = picker(mode)
    for frame in range(400):
      gas = 20 <= frame < 120  # Gas Override Boost only builds while the driver presses the gas
      out = boost.update(make_sm(gas_pressed=gas, gas=0.9, mph=40), 0.5, 1.5, 1.5)
      assert boost.applied == pytest.approx(out - 0.5, abs=1e-6)
      assert boost.applied >= 0.0
    assert (boost.applied > 0.0) == (mode != MODE_OFF)
    assert boost.acceleration_control_boost == pytest.approx(boost.applied if mode == MODE_AP_ACCEL_CONTROL else 0.0)

  def test_boost_max_per_mode(self):
    assert boost_max(MODE_OFF, P.aggressive) == 0.0
    assert boost_max(1, P.relaxed) == 0.0  # the removed Gas mode
    for idx, personality in enumerate((P.aggressive, P.standard, P.relaxed)):
      assert boost_max(MODE_SUNNY, personality) == SUNNY_MAX[idx]
      assert boost_max(MODE_EVP_SET_SPEED, personality) == EVP_MAX[idx]
      assert boost_max(MODE_AP_ACCEL_CONTROL, personality) == ACCELERATION_CONTROL_BOOST[personality]
      bias, gain = get_accel_boost(personality)
      assert boost_max(MODE_OP_PERSONALITY, personality) == bias + gain * OP_BAR_MODEL_ACCEL
    assert boost_max(MODE_AP_ACCEL_CONTROL, SimpleNamespace(raw=P.relaxed)) == 0.1  # a capnp enum


class TestOneAtATime:
  def test_modes_never_stack(self):
    for mode in MODES:
      boost = picker(mode)
      run(boost, 10, mph=40)
      run(boost, 2000, gas_pressed=True, mph=40)  # every boost would want to add something here
      others = {MODE_SUNNY: boost.sunny.total_boost, MODE_EVP_SET_SPEED: boost.set_speed.total_boost,
                MODE_AP_ACCEL_CONTROL: boost.ap_accel_control.boost, MODE_OP_PERSONALITY: boost.op_personality.op.bias_filter.x}
      assert all(v == 0.0 for m, v in others.items() if m != mode), (mode, others)

  def test_switching_mode_starts_clean(self):
    params = FakeParams(MODE_SUNNY)
    boost = FmAccelBoostMode(CP, params)
    run(boost, 400)
    assert boost.applied > 0.0
    params.values[PARAM_MODE] = MODE_OFF
    out = run(boost, READ_FRAMES)
    assert boost.mode == MODE_OFF and out == 0.5
    params.values[PARAM_MODE] = MODE_SUNNY
    run(boost, READ_FRAMES, brake=0.5)  # the model predicts a brake press, so a new boost can't build
    assert boost.mode == MODE_SUNNY
    assert boost.applied == 0.0 and boost.sunny.boost == 0.0  # the old Sunny Boost total did not come back

  def test_switching_away_from_evp_and_back_starts_clean(self):
    params = FakeParams(MODE_EVP_SET_SPEED)
    boost = FmAccelBoostMode(CP, params)
    run(boost, 400, mph=40)
    assert boost.applied > 0.0
    params.values[PARAM_MODE] = MODE_OFF
    run(boost, READ_FRAMES, mph=40)
    params.values[PARAM_MODE] = MODE_EVP_SET_SPEED
    run(boost, READ_FRAMES, mph=40, e2e=-0.6)  # the model brakes, so a new boost can't build
    assert boost.mode == MODE_EVP_SET_SPEED and boost.applied == 0.0

  def test_mode_is_read_every_params_period(self):
    params = FakeParams(MODE_OFF)
    boost = FmAccelBoostMode(CP, params)
    run(boost, 1)
    params.values[PARAM_MODE] = MODE_OP_PERSONALITY
    run(boost, READ_FRAMES - 1)
    assert boost.mode == MODE_OFF
    run(boost, 1)
    assert boost.mode == MODE_OP_PERSONALITY


class TestPickerDecidesAlone(OpenpilotTestCase):
  """On real Params: sunnypilot's AccelBoostSP and AP's AccelerationControl toggles, on or off, change nothing."""
  def _drive(self, mode, sp, ac, frames=3 * READ_FRAMES):
    params = Params()
    params.put(PARAM_MODE, mode, block=True)
    params.put_bool("AccelBoostSP", sp, block=True)
    params.put_bool("AccelerationControl", ac, block=True)
    boost = FmAccelBoostMode(CP)
    outs = []
    for frame in range(frames):
      gas = 40 <= frame < 120
      outs.append(boost.update(make_sm(gas_pressed=gas, mph=40, personality=P.standard, desired_accel=0.1), 0.1, 1.5, 1.5))
    return boost, outs

  def test_toggles_change_nothing(self):
    for mode in MODES:
      runs = [self._drive(mode, sp, ac)[1] for sp in (False, True) for ac in (False, True)]
      assert all(r == runs[0] for r in runs), mode

  def test_toggles_on_with_off_add_nothing(self):
    boost, outs = self._drive(MODE_OFF, True, True)
    assert all(o == 0.1 for o in outs)
    assert boost.applied == 0.0 and boost.acceleration_control_boost == 0.0

  def test_ap_accel_control_runs_with_its_toggle_off(self):
    boost, outs = self._drive(MODE_AP_ACCEL_CONTROL, False, False)
    assert outs[-1] == pytest.approx(0.1 + ACCELERATION_CONTROL_BOOST[P.standard])

  def test_sunny_boost_runs_with_its_toggle_off(self):
    boost, outs = self._drive(MODE_SUNNY, False, False)
    assert boost.applied > 0.0

  def test_mode_from_real_params(self):
    params = Params()
    for mode in MODES:
      params.put(PARAM_MODE, mode, block=True)
      assert FmAccelBoostMode(CP).mode == mode
    params.remove(PARAM_MODE)
    assert FmAccelBoostMode(CP).mode == MODE_OFF  # fresh install


class _NoGasPress:
  """Stands in for comma's Gas Override Boost (removed from FM Pilot) in frames without a gas press, where it added
  nothing: it only built while the driver pressed the gas."""
  @staticmethod
  def update(sm, a_target, mpc, cruise):
    assert not sm['carState'].gasPressed
    return a_target


def _base_chain(sm, raw, mpc, cruise, gas, sunny, ac, sp_on, throttle_scale=1.0):
  """The base planner's boost step before the picker (fm-dev-sync 4b46d9e): Acceleration Control first, then comma's Gas
  Override Boost, or sunnypilot's Acceleration Boost in its place while that is on and Acceleration Control is off."""
  lifted = ac.update(sm, raw, throttle_scale)
  boost = sunny if (sp_on and not ac.enabled) else gas
  return boost.update(sm, lifted, mpc, cruise)


class _Bool:
  def __init__(self, value):
    self.value = value

  def get_bool(self, key):
    return self.value


def _random_frames(seed, n=600, gas_presses=True):
  rng = random.Random(seed)
  frames = []
  gas = False
  for _ in range(n):
    if gas_presses and rng.random() < 0.02:
      gas = not gas
    raw = rng.uniform(-1.5, 1.0)
    frames.append((make_sm(enabled=rng.random() > 0.02, gas_pressed=gas, mph=rng.uniform(0, 70), brake=rng.uniform(0, 0.4),
                           gas=rng.uniform(0, 1), should_stop=rng.random() < 0.05, curvature=rng.uniform(-0.003, 0.003),
                           personality=rng.choice((P.aggressive, P.standard, P.relaxed)), desired_accel=raw),
                   raw, rng.uniform(-1, 1.5), rng.uniform(-0.5, 1.5)))
  return frames


class TestSameAsBase:
  """Off adds nothing; and where the base ran exactly one boost, the picker's mode gives the base's numbers."""
  @pytest.mark.parametrize("seed", range(5))
  def test_off_is_the_model_target_unchanged(self, seed):
    boost = picker(MODE_OFF)
    for sm, raw, mpc, cruise in _random_frames(seed):
      assert boost.update(sm, raw, mpc, cruise) == raw
      assert boost.applied == 0.0

  @pytest.mark.parametrize("seed", range(5))
  def test_off_equals_the_base_whenever_the_base_added_nothing(self, seed):
    # without a gas override and with both toggles off, the base's chain adds nothing either
    gas, sunny, ac = _NoGasPress(), AccelBoostSP(), AccelerationControl(CP, params=_Bool(False))
    boost = picker(MODE_OFF)
    for sm, raw, mpc, cruise in _random_frames(seed, gas_presses=False):
      assert boost.update(sm, raw, mpc, cruise) == _base_chain(sm, raw, mpc, cruise, gas, sunny, ac, sp_on=False)

  @pytest.mark.parametrize("seed", range(5))
  def test_sunny_equals_the_base_with_acceleration_boost_on(self, seed):
    gas, sunny, ac = _NoGasPress(), AccelBoostSP(), AccelerationControl(CP, params=_Bool(False))
    boost = picker(MODE_SUNNY)
    for sm, raw, mpc, cruise in _random_frames(seed):
      assert boost.update(sm, raw, mpc, cruise) == _base_chain(sm, raw, mpc, cruise, gas, sunny, ac, sp_on=True)

  @pytest.mark.parametrize("seed", range(5))
  def test_ap_equals_the_base_with_acceleration_control_on_without_a_gas_override(self, seed):
    # the base also ran Gas Override Boost on top of Acceleration Control; with no gas press it adds nothing
    gas, sunny, ac = _NoGasPress(), AccelBoostSP(), AccelerationControl(CP, params=_Bool(True))
    boost = picker(MODE_AP_ACCEL_CONTROL)
    for sm, raw, mpc, cruise in _random_frames(seed, gas_presses=False):
      assert boost.update(sm, raw, mpc, cruise, 0.7) == pytest.approx(_base_chain(sm, raw, mpc, cruise, gas, sunny, ac, True, 0.7),
                                                                       abs=1e-12)
