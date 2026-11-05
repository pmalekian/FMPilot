"""FM Pilot: lane keeping mode (fm_lane_keep_mode.py) and its one-time seed from the older toggles."""
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.fm_lane_keep_mode import (LEGACY_CENTERING, LEGACY_GUARD, MODE_BLENDFUL,
                                                                          MODE_CENTERING, MODE_GUARD, MODE_OFF, PARAM_MODE,
                                                                          get_lane_keep_mode, migrate_lane_keep_mode,
                                                                          set_lane_centering)
from openpilot.sunnypilot.system.params_migration import _migrate_fm_lane_keep_mode


class FakeParams:
  def __init__(self, **values):
    self.values = dict(values)

  def get(self, key, block=False, return_default=False):
    return self.values.get(key)

  def get_bool(self, key, block=False):
    return bool(self.values.get(key, False))

  def put(self, key, value, block=False):
    self.values[key] = value

  def put_bool(self, key, value, block=False):
    self.values[key] = bool(value)


# Every combination the older toggles can be in, and the mode it must come out as
LEGACY_CASES = [
  ({}, MODE_OFF),
  ({LEGACY_CENTERING: False, LEGACY_GUARD: False}, MODE_OFF),
  ({LEGACY_CENTERING: True, LEGACY_GUARD: False}, MODE_CENTERING),  # Frankie's setup
  ({LEGACY_CENTERING: False, LEGACY_GUARD: True}, MODE_GUARD),
  ({LEGACY_CENTERING: True, LEGACY_GUARD: True}, MODE_CENTERING),  # lane centering won before, so it wins here
]


@pytest.mark.parametrize("legacy,expected", LEGACY_CASES)
def test_unset_mode_follows_older_toggles(legacy, expected):
  assert get_lane_keep_mode(FakeParams(**legacy)) == expected


@pytest.mark.parametrize("legacy,expected", LEGACY_CASES)
def test_migration_seeds_the_same_mode_once(legacy, expected):
  params = FakeParams(**legacy)
  assert migrate_lane_keep_mode(params)
  assert params.values[PARAM_MODE] == expected
  # older toggles are left as they were
  for key, value in legacy.items():
    assert params.values[key] == value
  # a second boot doesn't touch it, even if the older toggles change
  params.values[LEGACY_CENTERING] = not params.values.get(LEGACY_CENTERING, False)
  assert not migrate_lane_keep_mode(params)
  assert params.values[PARAM_MODE] == expected


@pytest.mark.parametrize("mode", [MODE_OFF, MODE_CENTERING, MODE_GUARD, MODE_BLENDFUL])
def test_set_mode_wins_over_older_toggles(mode):
  params = FakeParams(**{PARAM_MODE: mode, LEGACY_CENTERING: True, LEGACY_GUARD: True})
  assert get_lane_keep_mode(params) == mode
  assert not migrate_lane_keep_mode(params)
  assert params.values[PARAM_MODE] == mode


@pytest.mark.parametrize("bad", [7, -1, "x", ""])
def test_unexpected_value_falls_back_and_is_reseeded(bad):
  params = FakeParams(**{PARAM_MODE: bad, LEGACY_CENTERING: True})
  assert get_lane_keep_mode(params) == MODE_CENTERING
  assert migrate_lane_keep_mode(params)
  assert params.values[PARAM_MODE] == MODE_CENTERING


def test_params_migration_entry_point_seeds_it():
  params = FakeParams(**{LEGACY_CENTERING: True})
  _migrate_fm_lane_keep_mode(params)
  assert params.values[PARAM_MODE] == MODE_CENTERING


def test_params_migration_never_raises():
  class Broken(FakeParams):
    def get(self, *a, **k):
      raise RuntimeError("boom")
  _migrate_fm_lane_keep_mode(Broken())


def test_device_toggle_sets_mode():
  params = FakeParams(**{PARAM_MODE: MODE_OFF})
  set_lane_centering(params, True)
  assert get_lane_keep_mode(params) == MODE_CENTERING
  set_lane_centering(params, False)
  assert get_lane_keep_mode(params) == MODE_OFF


def test_device_toggle_off_leaves_other_modes_alone():
  for mode in (MODE_GUARD, MODE_BLENDFUL):
    params = FakeParams(**{PARAM_MODE: mode})
    set_lane_centering(params, False)
    assert get_lane_keep_mode(params) == mode


def test_real_params_key_has_no_default():
  # The seed relies on the key being unset until first written; a default would hide the older toggles
  from openpilot.common.params import Params
  assert Params().get_default_value(PARAM_MODE) is None
