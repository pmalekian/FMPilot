"""FM Pilot: modeld_v2's per-model settings (FM_LateralSmoothingEnabled, FM_LowSpeedCurvatureFloor) with a Jetlink
joining model in the loop.

With Jetlink on, the model modeld_v2's loop runs is jetlink's JoiningModelState. A write on it stays on the wrapper
(jetlink only gives setters to lat_delay, PLANPLUS_CONTROL and frame_drop_ratio) and a read follows the model that is
driving. The loop used to do `model.LAT_SMOOTH_SECONDS = f(params, model.BUNDLE_LAT_SMOOTH_SECONDS)`:
  - the small model never got the setting (its curvature low-pass stayed at the bundle's value while the delay handed
    to the model used the setting's), and
  - once the big model drove, the read of BUNDLE_LAT_SMOOTH_SECONDS, which only modeld_v2's ModelState has, was an
    AttributeError on the frame thread: modeld down within 3 s of every swap.
"""
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

from jetlink.openpilot.joining import JoiningModelState

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.modeld_v2.modeld import (ACTION_SPEED_FLOOR, ACTION_SPEED_FLOOR_LOW_SPEED_FIX, ModelState,
                                                   fm_apply_model_settings, fm_own_model_states, fm_set_action_floor)


class FakeParams:
  def __init__(self, enabled=True, seconds=0.2):
    self.values = {"FM_LateralSmoothingEnabled": enabled, "FM_LateralSmoothingSeconds": seconds}

  def get_bool(self, key):
    return self.values[key]

  def get(self, key, return_default=False):
    return self.values[key]


def model_state(bundle_seconds: float) -> ModelState:
  """A modeld_v2 ModelState without loading a model: only what the settings touch."""
  m = ModelState.__new__(ModelState)
  m.BUNDLE_LAT_SMOOTH_SECONDS = bundle_seconds
  m.LAT_SMOOTH_SECONDS = bundle_seconds
  m.action_speed_floor = ACTION_SPEED_FLOOR
  m.lat_delay = 0.25
  m.PLANPLUS_CONTROL = 1.0
  return m


class TestFmModelSettingsWithJetlink(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.stop = threading.Event()

    def connect(should_stop=None):
      self.stop.wait(5)
      raise ConnectionError("no link in this test")

    self.small = model_state(0.0)  # e.g. Cheddar: the bundle says unsmoothed
    self.joining = JoiningModelState(self.small, connect, build=None, progress=mock.MagicMock(), log=mock.MagicMock())

  def tearDown(self):
    self.stop.set()
    self.joining.close()
    super().tearDown()

  def big_drives(self):
    """Jetlink's large model state as the one driving: comma's own smoothing, no bundle value."""
    big = SimpleNamespace(LAT_SMOOTH_SECONDS=0.0, LONG_SMOOTH_SECONDS=0.0, lat_delay=0.25, PLANPLUS_CONTROL=1.0, chestnut=True)
    self.joining._active = big
    return big

  def test_the_setting_reaches_the_small_model_behind_the_joining_model(self):
    fm_apply_model_settings(FakeParams(True, 0.2), self.joining, self.small, ACTION_SPEED_FLOOR_LOW_SPEED_FIX)
    self.assertEqual(self.small.LAT_SMOOTH_SECONDS, 0.2)  # the low-pass in get_action_from_model reads this one
    self.assertEqual(self.small.action_speed_floor, ACTION_SPEED_FLOOR_LOW_SPEED_FIX)
    self.assertEqual(self.joining.LAT_SMOOTH_SECONDS, 0.2)  # what the loop adds to the lateral delay: the same value
    # nothing was left on the wrapper to shadow the model that drives
    self.assertNotIn("LAT_SMOOTH_SECONDS", vars(self.joining))
    self.assertNotIn("action_speed_floor", vars(self.joining))

  def test_setting_off_restores_the_bundle_value(self):
    fm_apply_model_settings(FakeParams(True, 0.2), self.joining, self.small, ACTION_SPEED_FLOOR)
    fm_apply_model_settings(FakeParams(False, 0.2), self.joining, self.small, ACTION_SPEED_FLOOR)
    self.assertEqual(self.small.LAT_SMOOTH_SECONDS, 0.0)
    self.assertEqual(self.joining.LAT_SMOOTH_SECONDS, 0.0)

  def test_the_big_model_driving_does_not_raise_and_keeps_its_own_smoothing(self):
    big = self.big_drives()
    big.LAT_SMOOTH_SECONDS = 0.05  # not a value the setting can produce, to tell the two apart
    # what the loop did before: this read is the AttributeError that took modeld down after a swap
    with self.assertRaises(AttributeError):
      _ = self.joining.BUNDLE_LAT_SMOOTH_SECONDS
    fm_apply_model_settings(FakeParams(True, 0.2), self.joining, self.small, ACTION_SPEED_FLOOR)
    self.assertEqual(self.joining.LAT_SMOOTH_SECONDS, 0.05)  # the loop's read follows the big model
    self.assertEqual(big.LAT_SMOOTH_SECONDS, 0.05)
    self.assertEqual(self.small.LAT_SMOOTH_SECONDS, 0.2)  # ready for the hand-back
    self.assertFalse(hasattr(big, "action_speed_floor"))

  def test_the_per_frame_floor_goes_to_the_small_model_and_never_the_wrapper_or_a_big_model(self):
    big = self.big_drives()
    fm_set_action_floor(self.joining, self.small, ACTION_SPEED_FLOOR_LOW_SPEED_FIX)  # FM_StraightPullAwayFloor, every frame
    self.assertEqual(self.small.action_speed_floor, ACTION_SPEED_FLOOR_LOW_SPEED_FIX)
    self.assertNotIn("action_speed_floor", vars(self.joining))
    self.assertFalse(hasattr(big, "action_speed_floor"))
    fm_set_action_floor(self.joining, self.small, ACTION_SPEED_FLOOR)
    self.assertEqual(self.small.action_speed_floor, ACTION_SPEED_FLOOR)

  def test_own_model_states(self):
    big = model_state(0.1)
    self.assertEqual(fm_own_model_states(self.small, self.small), [self.small])  # no chestnut, no Jetlink: one model
    self.assertEqual(fm_own_model_states(big, self.small), [big, self.small])  # a chestnut: both are modeld_v2's
    self.assertEqual(fm_own_model_states(self.joining, self.small), [self.small])  # Jetlink
    self.assertEqual(fm_own_model_states(self.joining, None), [])
    self.assertEqual(fm_own_model_states(big, None), [big])  # big model only (no small model was built)

  def test_a_chestnut_pair_each_gets_its_own_bundle_value_when_off(self):
    big = model_state(0.1)
    fm_apply_model_settings(FakeParams(False), big, self.small, ACTION_SPEED_FLOOR)
    self.assertEqual((big.LAT_SMOOTH_SECONDS, self.small.LAT_SMOOTH_SECONDS), (0.1, 0.0))
    fm_apply_model_settings(FakeParams(True, 0.1), big, self.small, ACTION_SPEED_FLOOR_LOW_SPEED_FIX)
    self.assertEqual((big.LAT_SMOOTH_SECONDS, self.small.LAT_SMOOTH_SECONDS), (0.1, 0.1))
    self.assertEqual((big.action_speed_floor, self.small.action_speed_floor), (2.5, 2.5))


if __name__ == '__main__':
  unittest.main()
