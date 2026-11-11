from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.fm_boost_mode import (FmBoostMode, PARAM_MODE, MODE_OFF, MODE_GAS, MODE_PREDICTIVE,
                                                            MODE_SET_SPEED, DEFAULT_MODE, parse_mode)
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.accel_boost import AccelBoostSP, ACCEL_BOOST_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.set_speed_assist import MAX_BOOST as SSA_MAX

P = log.LongitudinalPersonality
READ_FRAMES = int(PARAMS_UPDATE_PERIOD / DT_MDL)


class FakeParams:
  def __init__(self, mode=None):
    self.values = {} if mode is None else {PARAM_MODE: mode}

  def get(self, key, return_default=False):
    return self.values.get(key, DEFAULT_MODE if return_default else None)


def make_sm(enabled=True, gas_pressed=False, mph=30, brake=0.0, gas=0.0, should_stop=False, curvature=0.0,
            personality=P.aggressive, probs=6, a_ego=0.0, lead=False):
  meta = SimpleNamespace(disengagePredictions=SimpleNamespace(brakePressProbs=[brake] * probs, gasPressProbs=[gas] * probs))
  return {
    'selfdriveState': SimpleNamespace(enabled=enabled, personality=personality),
    'carState': SimpleNamespace(gasPressed=gas_pressed, vEgo=mph * CV.MPH_TO_MS, aEgo=a_ego),
    'modelV2': SimpleNamespace(meta=meta, action=SimpleNamespace(shouldStop=should_stop, desiredCurvature=curvature)),
    'radarState': SimpleNamespace(leadOne=SimpleNamespace(present=lead)),
  }


def run(boost, frames, e2e=0.5, **kwargs):
  out = None
  for _ in range(frames):
    # model wants 0.5 m/s^2, classic planner and cruise allow 1.5: the model is limiting
    out = boost.update(make_sm(**kwargs), e2e, 1.5, 1.5)
  return out


class TestParseMode:
  @pytest.mark.parametrize("value,mode", [(0, MODE_OFF), (1, MODE_GAS), (2, MODE_PREDICTIVE), ("2", MODE_PREDICTIVE),
                                          (3, MODE_SET_SPEED), ("3", MODE_SET_SPEED),
                                          (None, DEFAULT_MODE), (7, DEFAULT_MODE), ("x", DEFAULT_MODE)])
  def test_parse(self, value, mode):
    assert parse_mode(value) == mode

  def test_default_is_gas_like_before(self):
    assert DEFAULT_MODE == MODE_GAS
    assert FmBoostMode(FakeParams()).mode == MODE_GAS


class TestPredictive:
  def test_no_reason_to_slow_builds_to_personality_max(self):
    for personality, cap in zip((P.aggressive, P.standard, P.relaxed), ACCEL_BOOST_MAX, strict=True):
      boost = AccelBoostSP()
      out = run(boost, 400, personality=personality)
      assert out == pytest.approx(0.5 + cap)

  def test_build_rate_is_limited(self):
    boost = AccelBoostSP()
    run(boost, 1)
    assert boost.total_boost == pytest.approx(0.2 * DT_MDL)

  def test_model_wants_to_brake_no_boost(self):
    boost = AccelBoostSP()
    assert run(boost, 400, brake=0.5) == 0.5

  def test_strong_gas_prediction_overrides_brake_prediction(self):
    boost = AccelBoostSP()
    assert run(boost, 400, brake=0.5, gas=0.9) > 0.5

  def test_should_stop_no_boost(self):
    boost = AccelBoostSP()
    assert run(boost, 400, should_stop=True) == 0.5

  def test_fades_in_tight_curves(self):
    v = 30 * CV.MPH_TO_MS
    boost = AccelBoostSP()
    assert run(boost, 400, curvature=2.5 / v ** 2) == 0.5

  def test_no_boost_when_model_brakes(self):
    boost = AccelBoostSP()
    run(boost, 400)
    assert run(boost, 1, e2e=-0.8) == -0.8

  def test_no_boost_near_standstill(self):
    boost = AccelBoostSP()
    assert run(boost, 400, mph=1) == 0.5

  def test_disengaged_resets(self):
    boost = AccelBoostSP()
    run(boost, 400)
    assert run(boost, 1, enabled=False) == 0.5
    assert boost.boost == 0.0

  def test_model_without_press_outputs_no_boost(self):
    boost = AccelBoostSP()
    assert run(boost, 400, probs=0) == 0.5


class TestExclusive:
  def test_off_never_boosts(self):
    boost = FmBoostMode(FakeParams(MODE_OFF))
    run(boost, 10)
    assert run(boost, 2000, gas_pressed=True) == 0.5
    assert boost.total_boost == 0.0

  def test_gas_mode_ignores_predictions(self):
    boost = FmBoostMode(FakeParams(MODE_GAS))
    assert run(boost, 400) == 0.5  # no gas press, predictions say go: Gas Boost does nothing

  def test_predictive_mode_ignores_gas_press(self):
    boost = FmBoostMode(FakeParams(MODE_PREDICTIVE))
    run(boost, 10, brake=0.5)
    assert run(boost, 2000, brake=0.5, gas_pressed=True) == 0.5  # model wants to slow: no boost, even with gas pressed

  def test_never_stacks(self):
    boost = FmBoostMode(FakeParams(MODE_PREDICTIVE))
    out = run(boost, 2000, gas_pressed=True)
    assert out == pytest.approx(0.5 + ACCEL_BOOST_MAX[0])
    assert boost.gas.total_boost == 0.0

  def test_switching_mode_starts_clean(self):
    params = FakeParams(MODE_GAS)
    boost = FmBoostMode(params)
    run(boost, 10)
    run(boost, 2000, gas_pressed=True)
    assert boost.total_boost > 0.0
    params.values[PARAM_MODE] = MODE_OFF
    out = run(boost, READ_FRAMES)
    assert boost.mode == MODE_OFF
    assert out == 0.5
    params.values[PARAM_MODE] = MODE_GAS
    run(boost, READ_FRAMES)
    assert boost.mode == MODE_GAS
    assert boost.total_boost == 0.0  # the old Gas Boost total did not come back


# FM Pilot: what the boost added, for the log and the on-screen bar

@pytest.mark.parametrize("mode", [MODE_OFF, MODE_GAS, MODE_PREDICTIVE, MODE_SET_SPEED])
def test_applied_is_what_was_added(mode):
  boost = FmBoostMode(FakeParams(mode))
  for frame in range(400):
    gas = 20 <= frame < 120  # Gas Boost only builds while the driver presses the gas
    out = boost.update(make_sm(gas_pressed=gas, gas=0.9), 0.5, 1.5, 1.5)
    assert boost.applied == pytest.approx(out - 0.5, abs=1e-6)
    assert boost.applied >= 0.0
  if mode == MODE_OFF:
    assert boost.applied == 0.0
  else:
    assert boost.applied > 0.0


def test_boost_max_per_mode():
  from openpilot.selfdrive.controls.lib.accel_boost import ACCEL_BOOST_MAX as GAS_MAX
  from openpilot.selfdrive.controls.lib.fm_boost_mode import boost_max
  assert boost_max(MODE_OFF, P.aggressive) == 0.0
  assert boost_max(MODE_GAS, P.relaxed) == GAS_MAX
  assert boost_max(MODE_PREDICTIVE, P.aggressive) == ACCEL_BOOST_MAX[0]
  assert boost_max(MODE_PREDICTIVE, P.standard) == ACCEL_BOOST_MAX[1]
  assert boost_max(MODE_PREDICTIVE, P.relaxed) == ACCEL_BOOST_MAX[2]
  assert boost_max(MODE_SET_SPEED, P.aggressive) == SSA_MAX[0]
  assert boost_max(MODE_SET_SPEED, P.standard) == SSA_MAX[1]
  assert boost_max(MODE_SET_SPEED, P.relaxed) == SSA_MAX[2]


# FM Pilot: Set Speed Assist as mode 3

class TestSetSpeed:
  def test_set_speed_mode_boosts_toward_cruise_at_highway_speed(self):
    boost = FmBoostMode(FakeParams(MODE_SET_SPEED))
    out = run(boost, 400, mph=40, brake=0.5)  # predictions say slow: Set Speed doesn't use them
    assert out == pytest.approx(0.5 + SSA_MAX[0])
    assert boost.gas.total_boost == 0.0 and boost.predictive.total_boost == 0.0  # never stacks

  def test_set_speed_mode_waits_above_20_mph(self):
    boost = FmBoostMode(FakeParams(MODE_SET_SPEED))
    assert run(boost, 400, mph=15) == 0.5

  def test_other_modes_never_run_set_speed(self):
    for mode in (MODE_OFF, MODE_GAS, MODE_PREDICTIVE):
      boost = FmBoostMode(FakeParams(mode))
      run(boost, 400, mph=40)
      assert boost.set_speed.total_boost == 0.0

  def test_switching_away_and_back_starts_clean(self):
    params = FakeParams(MODE_SET_SPEED)
    boost = FmBoostMode(params)
    run(boost, 400, mph=40)
    assert boost.total_boost > 0.0
    params.values[PARAM_MODE] = MODE_OFF
    run(boost, READ_FRAMES, mph=40)
    assert boost.mode == MODE_OFF
    params.values[PARAM_MODE] = MODE_SET_SPEED
    run(boost, READ_FRAMES, mph=40, e2e=-0.6)  # the model brakes, so a new boost can't build: what's left is carried over
    assert boost.mode == MODE_SET_SPEED
    assert boost.total_boost == 0.0
