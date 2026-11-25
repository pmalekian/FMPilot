"""FM Pilot: lane keeping mode (fm_lane_keep_mode.py) and its one-time seed from the older toggles (ported from
fm-dev-nav-settle; fm-dev-sync adds sunnypilot's Blendful toggle as the last older toggle and a default of Off, so the
seed writes nothing when they pick Off)."""
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.fm_lane_keep_mode import (LEGACY_BLENDFUL, LEGACY_CENTERING, LEGACY_GUARD,
                                                                          MODE_BLENDFUL,
                                                                          MODE_CENTERING, MODE_GUARD, MODE_OFF, PARAM_MODE,
                                                                          get_lane_keep_mode, migrate_lane_keep_mode,
                                                                          set_lane_centering)
from openpilot.sunnypilot.system import fm_migration


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
  ({LEGACY_BLENDFUL: True}, MODE_BLENDFUL),  # sunnypilot's own Blendful toggle (hidden on fm-dev-sync)
  ({LEGACY_GUARD: True, LEGACY_BLENDFUL: True}, MODE_GUARD),
  ({LEGACY_CENTERING: True, LEGACY_BLENDFUL: True}, MODE_CENTERING),
]


@pytest.mark.parametrize("legacy,expected", LEGACY_CASES)
def test_unset_mode_follows_older_toggles(legacy, expected):
  assert get_lane_keep_mode(FakeParams(**legacy)) == expected


@pytest.mark.parametrize("legacy,expected", LEGACY_CASES)
def test_migration_seeds_the_same_mode_once(legacy, expected):
  params = FakeParams(**legacy)
  assert migrate_lane_keep_mode(params) == (expected != MODE_OFF)  # Off is the default: nothing written
  assert params.values.get(PARAM_MODE, MODE_OFF) == expected
  # older toggles are left as they were
  for key, value in legacy.items():
    assert params.values[key] == value
  # a second boot doesn't touch it, even if the older toggles change
  if expected == MODE_OFF:
    return
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


def _lane_keep_migration():
  return [m for m in fm_migration.MIGRATIONS if m[0] == "lane_keep_mode"]


def test_the_one_time_migration_seeds_it():
  params = FakeParams(**{LEGACY_CENTERING: True})
  assert fm_migration.run(params, _lane_keep_migration()) == ["lane_keep_mode"]
  assert params.values[PARAM_MODE] == MODE_CENTERING


def test_his_blendful_is_kept():
  params = FakeParams(**{PARAM_MODE: MODE_BLENDFUL, LEGACY_CENTERING: True})
  fm_migration.run(params, _lane_keep_migration())
  assert params.values[PARAM_MODE] == MODE_BLENDFUL


def test_a_failing_migration_never_raises():
  class Broken(FakeParams):
    def get(self, key, *a, **k):
      if key == fm_migration.FLAG:
        return None
      raise RuntimeError("boom")
  assert fm_migration.run(Broken(), _lane_keep_migration()) == []  # not marked done, so it retries next boot


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


def test_real_params_key_defaults_to_off():
  # the seed runs before manager writes defaults, so a default of Off doesn't hide the older toggles
  from openpilot.common.params import Params
  assert Params().get(PARAM_MODE, return_default=True) == MODE_OFF
