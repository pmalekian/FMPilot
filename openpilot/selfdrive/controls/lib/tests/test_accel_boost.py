from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost, ACCEL_BOOST_MAX, ACCEL_BOOST_PER_OVERRIDE


def make_sm(enabled=True, gas_pressed=False, v_ego=30 * CV.MPH_TO_MS):
  return {
    'selfdriveState': SimpleNamespace(enabled=enabled),
    'carState': SimpleNamespace(gasPressed=gas_pressed, vEgo=v_ego),
  }


def run(boost, frames, **kwargs):
  out = None
  for _ in range(frames):
    # model wants 0.5 m/s^2, classic planner and cruise allow 1.5: the model is limiting
    out = boost.update(make_sm(**kwargs), 0.5, 1.5, 1.5)
  return out


class TestAccelBoost:
  def test_no_gas_no_boost(self):
    boost = AccelBoost()
    assert run(boost, 200) == 0.5
    assert boost.total_boost == 0.0

  def test_gas_press_adds_capped_boost(self):
    boost = AccelBoost()
    run(boost, 10)  # latch eligibility before the press
    run(boost, 2000, gas_pressed=True)  # one long press
    assert 0.0 < boost.total_boost <= ACCEL_BOOST_PER_OVERRIDE + 1e-9

  def test_total_capped(self):
    boost = AccelBoost()
    for _ in range(20):  # many separate presses
      run(boost, 10)
      run(boost, 2000, gas_pressed=True)
    assert boost.total_boost <= ACCEL_BOOST_MAX + 1e-9
    assert run(boost, 1) <= 0.5 + ACCEL_BOOST_MAX + 1e-9

  def test_reset_on_disengage(self):
    boost = AccelBoost()
    run(boost, 10)
    run(boost, 2000, gas_pressed=True)
    assert boost.total_boost > 0.0
    run(boost, 1, enabled=False)
    assert boost.total_boost == 0.0

  def test_no_boost_at_low_speed(self):
    boost = AccelBoost()
    run(boost, 10, v_ego=5 * CV.MPH_TO_MS)
    run(boost, 2000, gas_pressed=True, v_ego=5 * CV.MPH_TO_MS)
    assert boost.total_boost == 0.0

  def test_never_changes_braking(self):
    boost = AccelBoost()
    run(boost, 10)
    run(boost, 2000, gas_pressed=True)
    # a hard braking request from the model passes through untouched
    assert boost.update(make_sm(), -2.0, 1.5, 1.5) == -2.0
