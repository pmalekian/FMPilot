"""FM Pilot: modeld_v2's per-model settings (FM_SmallModelLatSmoothing, FM_BigModelLatSmoothing) with a Jetlink
joining model in the loop.

With Jetlink on, the model modeld_v2's loop runs is jetlink's JoiningModelState. A write on it stays on the wrapper
(jetlink only gives setters to lat_delay, PLANPLUS_CONTROL and frame_drop_ratio) and a read follows the model that is
driving. Per-model settings must therefore be written to modeld_v2's own ModelStates and never to the wrapper.
"""
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

from jetlink.openpilot.joining import JoiningModelState

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.modeld_v2.fm_lateral_smoothing import (
  MODE_0_00,
  MODE_0_10,
  MODE_DEFAULT,
  PARAM_BIG_MODEL_SMOOTHING,
  PARAM_SMALL_MODEL_SMOOTHING,
)
from openpilot.sunnypilot.modeld_v2.modeld import (ACTION_SPEED_FLOOR, ACTION_SPEED_FLOOR_LOW_SPEED_FIX, ModelState,
                                                   fm_apply_model_settings, fm_own_model_states, fm_set_action_floor)


class FakeParams:
  def __init__(self, small_mode=MODE_DEFAULT, big_mode=MODE_DEFAULT):
    self.values = {PARAM_SMALL_MODEL_SMOOTHING: small_mode, PARAM_BIG_MODEL_SMOOTHING: big_mode}

  def get(self, key, return_default=False):
    return self.values[key]


def model_state(bundle_seconds: float, chestnut: bool = False) -> ModelState:
  """A modeld_v2 ModelState without loading a model: only what the settings touch."""
  m = ModelState.__new__(ModelState)
  m.BUNDLE_LAT_SMOOTH_SECONDS = bundle_seconds
  m.LAT_SMOOTH_SECONDS = bundle_seconds
  m.action_speed_floor = ACTION_SPEED_FLOOR
  m.chestnut = chestnut
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

    self.small = model_state(0.0, chestnut=False)  # e.g. Cheddar: the bundle says unsmoothed
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
    fm_apply_model_settings(FakeParams(small_mode=MODE_0_10), self.joining, self.small, ACTION_SPEED_FLOOR_LOW_SPEED_FIX)
    self.assertEqual(self.small.LAT_SMOOTH_SECONDS, 0.1)  # the low-pass in get_action_from_model reads this one
    self.assertEqual(self.small.action_speed_floor, ACTION_SPEED_FLOOR_LOW_SPEED_FIX)
    self.assertEqual(self.joining.LAT_SMOOTH_SECONDS, 0.1)  # what the loop adds to the lateral delay: the same value
    # nothing was left on the wrapper to shadow the model that drives
    self.assertNotIn("LAT_SMOOTH_SECONDS", vars(self.joining))
    self.assertNotIn("action_speed_floor", vars(self.joining))

  def test_model_default_restores_the_bundle_value(self):
    fm_apply_model_settings(FakeParams(small_mode=MODE_0_10), self.joining, self.small, ACTION_SPEED_FLOOR)
    fm_apply_model_settings(FakeParams(small_mode=MODE_DEFAULT), self.joining, self.small, ACTION_SPEED_FLOOR)
    self.assertEqual(self.small.LAT_SMOOTH_SECONDS, 0.0)
    self.assertEqual(self.joining.LAT_SMOOTH_SECONDS, 0.0)

  def test_the_big_model_driving_does_not_raise_and_keeps_its_own_smoothing(self):
    big = self.big_drives()
    big.LAT_SMOOTH_SECONDS = 0.05  # not a value the setting can produce, to tell the two apart
    with self.assertRaises(AttributeError):
      _ = self.joining.BUNDLE_LAT_SMOOTH_SECONDS
    fm_apply_model_settings(FakeParams(small_mode=MODE_0_10, big_mode=MODE_0_10), self.joining, self.small, ACTION_SPEED_FLOOR)
    self.assertEqual(self.joining.LAT_SMOOTH_SECONDS, 0.05)  # the loop's read follows the big model
    self.assertEqual(big.LAT_SMOOTH_SECONDS, 0.05)
    self.assertEqual(self.small.LAT_SMOOTH_SECONDS, 0.1)  # ready for the hand-back
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
    big = model_state(0.1, chestnut=True)
    self.assertEqual(fm_own_model_states(self.small, self.small), [self.small])  # no chestnut, no Jetlink: one model
    self.assertEqual(fm_own_model_states(big, self.small), [big, self.small])  # a chestnut: both are modeld_v2's
    self.assertEqual(fm_own_model_states(self.joining, self.small), [self.small])  # Jetlink
    self.assertEqual(fm_own_model_states(self.joining, None), [])
    self.assertEqual(fm_own_model_states(big, None), [big])  # big model only (no small model was built)

  def test_a_chestnut_pair_applies_small_and_big_settings_independently(self):
    big = model_state(0.1, chestnut=True)
    fm_apply_model_settings(FakeParams(MODE_DEFAULT, MODE_DEFAULT), big, self.small, ACTION_SPEED_FLOOR)
    self.assertEqual((big.LAT_SMOOTH_SECONDS, self.small.LAT_SMOOTH_SECONDS), (0.1, 0.0))
    # small = 0.10 s affects only small; big = 0.00 s clears big's bundle override
    fm_apply_model_settings(FakeParams(MODE_0_10, MODE_0_00), big, self.small, ACTION_SPEED_FLOOR_LOW_SPEED_FIX)
    self.assertEqual((big.LAT_SMOOTH_SECONDS, self.small.LAT_SMOOTH_SECONDS), (0.0, 0.1))
    self.assertEqual((big.action_speed_floor, self.small.action_speed_floor), (2.5, 2.5))
    # big = 0.10 s never shifts lat_delay (keeps bundle 0.0 if bundle was 0.0; post-action smooth_curvature adds 0.1 s)
    big_zero = model_state(0.0, chestnut=True)
    fm_apply_model_settings(FakeParams(MODE_0_00, MODE_0_10), big_zero, self.small, ACTION_SPEED_FLOOR)
    self.assertEqual((big_zero.LAT_SMOOTH_SECONDS, self.small.LAT_SMOOTH_SECONDS), (0.0, 0.0))


if __name__ == '__main__':
  unittest.main()
