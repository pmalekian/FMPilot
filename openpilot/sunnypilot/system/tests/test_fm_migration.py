"""FM Pilot: the one-time settings migration runner (sunnypilot/system/fm_migration.py).

Each feature PR that reshapes a setting adds its own mapping tests next to these, on real Params.
"""
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.system import fm_migration
from openpilot.sunnypilot.system.fm_migration import FLAG, done, put_if_unset, run


class TestRunner(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.params = Params()
    self.calls = []

  def m(self, name, fail=False):
    def fn(params):
      self.calls.append(name)
      if fail:
        raise RuntimeError("boom")
      return f"{name} done"
    return name, fn

  def test_each_runs_once(self):
    ms = [self.m("a"), self.m("b")]
    assert run(self.params, ms) == ["a", "b"]
    assert done(self.params) == {"a", "b"}
    assert run(self.params, ms) == []
    assert self.calls == ["a", "b"]

  def test_a_later_build_runs_only_the_new_one(self):
    run(self.params, [self.m("a")])
    assert run(self.params, [self.m("a"), self.m("b")]) == ["b"]
    assert self.calls == ["a", "b"]

  def test_a_failure_is_retried_next_boot(self):
    assert run(self.params, [self.m("a", fail=True), self.m("b")]) == ["b"]
    assert done(self.params) == {"b"}
    assert run(self.params, [self.m("a"), self.m("b")]) == ["a"]
    assert done(self.params) == {"a", "b"}

  def test_nothing_ran_writes_nothing(self):
    assert run(self.params, []) == []
    assert self.params.get(FLAG) is None

  def test_put_if_unset_keeps_a_chosen_value(self):
    assert put_if_unset(self.params, "LongitudinalPersonality", 0)
    assert not put_if_unset(self.params, "LongitudinalPersonality", 2)
    assert self.params.get("LongitudinalPersonality") == 0

  def test_registry_names_are_unique(self):
    names = [n for n, _ in fm_migration.MIGRATIONS]
    assert len(names) == len(set(names)) and all("," not in n for n in names)

  def test_registered_migrations_are_noops_on_a_fresh_install(self):
    """A fresh install has none of the old keys: every registered migration must leave all FM keys unset, so the
    stock/off defaults apply."""
    keys = [k.decode() if isinstance(k, bytes) else k for k in self.params.all_keys()]
    fm_keys = [k for k in keys if k.startswith("FM_") and k != FLAG]
    before = {k: self.params.get(k) for k in fm_keys}
    run(self.params)
    assert {k: self.params.get(k) for k in fm_keys} == before


class TestHook(OpenpilotTestCase):
  def test_params_migration_runs_it(self):
    from openpilot.sunnypilot.system import params_migration
    params = Params()
    seen = []
    orig = fm_migration.MIGRATIONS[:]
    fm_migration.MIGRATIONS.append(("test_hook", lambda p: seen.append(1)))
    try:
      params_migration.run_migration(params)
    finally:
      fm_migration.MIGRATIONS[:] = orig
    assert seen == [1] and "test_hook" in done(params)


class TestMaxTorque(OpenpilotTestCase):
  """Max Torque: Panda Allows Higher Torque off -> 385; on -> Max Torque on the 15 grid (580 -> 580)."""

  def migrate(self, **old):
    params = Params()
    for k in ("FM_CustomMaxTorque", "FM_PandaHighTorque", "FM_MaxTorque", FLAG):  # each call a fresh device
      params.remove(k)
    for k, v in old.items():
      params.put(k, v, block=True)
    run(params, [m for m in fm_migration.MIGRATIONS if m[0] == "max_torque"])
    return params

  def test_his_setup_carries_over(self):
    params = self.migrate(FM_PandaHighTorque=True, FM_MaxTorque=580, FM_TorqueFastRamp=True, FM_TorqueFullCapHighAngle=True,
                          FM_TorqueMode=1)
    assert params.get("FM_CustomMaxTorque") == 580

  def test_panda_off_is_385(self):
    assert self.migrate(FM_PandaHighTorque=False, FM_MaxTorque=580).get("FM_CustomMaxTorque") == 385

  def test_old_steps_round_to_the_15_grid(self):
    for old, new in ((405, 400), (430, 430), (455, 460), (480, 475), (505, 505), (530, 535), (555, 550), (580, 580)):
      assert self.migrate(FM_PandaHighTorque=True, FM_MaxTorque=old).get("FM_CustomMaxTorque") == new, old

  def test_panda_on_without_a_max_uses_the_old_default(self):
    assert self.migrate(FM_PandaHighTorque=True).get("FM_CustomMaxTorque") == 475

  def test_fresh_install_writes_nothing(self):
    params = self.migrate()
    assert params.get("FM_CustomMaxTorque") is None  # manager then writes the 385 default
    assert "max_torque" in done(params)

  def test_old_keys_untouched_and_a_chosen_value_kept(self):
    params = Params()
    params.put("FM_CustomMaxTorque", 430, block=True)
    for k, v in {"FM_PandaHighTorque": True, "FM_MaxTorque": 580}.items():
      params.put(k, v, block=True)
    assert run(params, [m for m in fm_migration.MIGRATIONS if m[0] == "max_torque"]) == ["max_torque"]  # it ran
    assert params.get("FM_CustomMaxTorque") == 430
    assert params.get("FM_PandaHighTorque") is True and params.get("FM_MaxTorque") == 580


class TestReleaseGuard(OpenpilotTestCase):
  """Release Guard: off -> Stock (0); on -> 75 or 150 (anything else or unset -> 150)."""

  def migrate(self, **old):
    params = Params()
    for k in ("FM_ReleaseGuardLimit", "FM_ReleaseGuard", "FM_ReleaseGuardMax", FLAG):
      params.remove(k)
    for k, v in old.items():
      params.put(k, v, block=True)
    assert run(params, [m for m in fm_migration.MIGRATIONS if m[0] == "release_guard"]) == ["release_guard"]
    return params

  def test_his_setup_carries_over(self):
    assert self.migrate(FM_ReleaseGuard=True, FM_ReleaseGuardMax=150).get("FM_ReleaseGuardLimit") == 150

  def test_75(self):
    assert self.migrate(FM_ReleaseGuard=True, FM_ReleaseGuardMax=75).get("FM_ReleaseGuardLimit") == 75

  def test_off_is_stock_whatever_the_level(self):
    assert self.migrate(FM_ReleaseGuard=False, FM_ReleaseGuardMax=75).get("FM_ReleaseGuardLimit") == 0

  def test_on_without_a_level_is_the_old_default(self):
    assert self.migrate(FM_ReleaseGuard=True).get("FM_ReleaseGuardLimit") == 150

  def test_fresh_install_writes_nothing(self):
    assert self.migrate().get("FM_ReleaseGuardLimit") is None

  def test_old_keys_untouched(self):
    params = self.migrate(FM_ReleaseGuard=True, FM_ReleaseGuardMax=75)
    assert params.get("FM_ReleaseGuard") is True and params.get("FM_ReleaseGuardMax") == 75


class TestLowSpeed(OpenpilotTestCase):
  """Feedforward: master off -> both Off, else per side; Low-Speed Correction Strength: off -> 100, else percent."""
  KEYS = ("FM_LowSpeedFFLeft", "FM_LowSpeedFFRight", "FM_LowSpeedCorrectionStrength", "FM_LowSpeedFFEnabled",
          "FM_LowSpeedFFStepLeft", "FM_LowSpeedFFStepRight", "FM_LowSpeedGainEnabled", "FM_LowSpeedGainPercent", FLAG)

  def migrate(self, **old):
    params = Params()
    for k in self.KEYS:
      params.remove(k)
    for k, v in old.items():
      params.put(k, v, block=True)
    names = ("low_speed_feedforward", "low_speed_correction")
    assert run(params, [m for m in fm_migration.MIGRATIONS if m[0] in names]) == list(names)
    return params

  def test_his_setup_carries_over(self):
    p = self.migrate(FM_LowSpeedFFEnabled=True, FM_LowSpeedFFStepLeft=1, FM_LowSpeedFFStepRight=2,
                     FM_LowSpeedGainEnabled=True, FM_LowSpeedGainPercent=70)
    assert (p.get("FM_LowSpeedFFLeft"), p.get("FM_LowSpeedFFRight"), p.get("FM_LowSpeedCorrectionStrength")) == (1, 2, 70)

  def test_master_off_is_both_off(self):
    p = self.migrate(FM_LowSpeedFFEnabled=False, FM_LowSpeedFFStepLeft=4, FM_LowSpeedFFStepRight=3)
    assert (p.get("FM_LowSpeedFFLeft"), p.get("FM_LowSpeedFFRight")) == (0, 0)

  def test_on_with_unset_steps_is_gentle(self):
    p = self.migrate(FM_LowSpeedFFEnabled=True)
    assert (p.get("FM_LowSpeedFFLeft"), p.get("FM_LowSpeedFFRight")) == (1, 1)

  def test_calmer_off_is_stock(self):
    assert self.migrate(FM_LowSpeedGainEnabled=False, FM_LowSpeedGainPercent=40).get("FM_LowSpeedCorrectionStrength") == 100

  def test_calmer_on_unset_is_70(self):
    assert self.migrate(FM_LowSpeedGainEnabled=True).get("FM_LowSpeedCorrectionStrength") == 70

  def test_fresh_install_writes_nothing(self):
    p = self.migrate()
    assert all(p.get(k) is None for k in ("FM_LowSpeedFFLeft", "FM_LowSpeedFFRight", "FM_LowSpeedCorrectionStrength"))

  def test_old_keys_untouched(self):
    p = self.migrate(FM_LowSpeedFFEnabled=False, FM_LowSpeedFFStepLeft=4, FM_LowSpeedGainEnabled=True, FM_LowSpeedGainPercent=55)
    assert (p.get("FM_LowSpeedFFEnabled"), p.get("FM_LowSpeedFFStepLeft"), p.get("FM_LowSpeedGainPercent")) == (False, 4, 55)


class TestLaneKeepMode(OpenpilotTestCase):
  """Lane Keeping: a mode set on fm-dev-nav-settle is kept; unset -> Lane Centering, then Lane Guard, then Blendful."""
  KEYS = ("FM_LaneKeepMode", "LaneCenteringEnabled", "LaneGuardEnabled", "Blendful", FLAG)

  def migrate(self, **old):
    params = Params()
    for k in self.KEYS:
      params.remove(k)
    for k, v in old.items():
      params.put(k, v, block=True)
    assert run(params, [m for m in fm_migration.MIGRATIONS if m[0] == "lane_keep_mode"]) == ["lane_keep_mode"]
    return params

  def test_his_setup_carries_over(self):
    assert self.migrate(FM_LaneKeepMode=3, LaneCenteringEnabled=True).get("FM_LaneKeepMode") == 3

  def test_legacy_centering(self):
    assert self.migrate(LaneCenteringEnabled=True, LaneGuardEnabled=True).get("FM_LaneKeepMode") == 1

  def test_legacy_guard(self):
    assert self.migrate(LaneGuardEnabled=True, Blendful=True).get("FM_LaneKeepMode") == 2

  def test_legacy_blendful(self):
    assert self.migrate(Blendful=True).get("FM_LaneKeepMode") == 3

  def test_fresh_install_writes_nothing(self):
    assert self.migrate().get("FM_LaneKeepMode") is None

  def test_old_keys_untouched(self):
    p = self.migrate(LaneGuardEnabled=True, Blendful=True)
    assert p.get("LaneGuardEnabled") is True and p.get("Blendful") is True


class TestModelLateralSmoothing(OpenpilotTestCase):
  """Small Model Smoothing: off -> Model Default (0), 0 -> 0.00 (1), 0.1/0.2 -> 0.10 (2); Big: off -> 0, on -> 2."""
  KEYS = ("FM_SmallModelLatSmoothing", "FM_BigModelLatSmoothing", "FM_LateralSmoothingEnabled",
          "FM_LateralSmoothingSeconds", "FM_BigModelSmoothing", FLAG)

  def migrate(self, **old):
    params = Params()
    for k in self.KEYS:
      params.remove(k)
    for k, v in old.items():
      params.put(k, v, block=True)
    names = ("small_model_lat_smoothing", "big_model_lat_smoothing")
    assert run(params, [m for m in fm_migration.MIGRATIONS if m[0] in names]) == list(names)
    return params

  def test_small_off_is_model_default(self):
    assert self.migrate(FM_LateralSmoothingEnabled=False, FM_LateralSmoothingSeconds=0.1).get("FM_SmallModelLatSmoothing") == 0

  def test_small_zero_maps_to_0_00(self):
    assert self.migrate(FM_LateralSmoothingEnabled=True, FM_LateralSmoothingSeconds=0.0).get("FM_SmallModelLatSmoothing") == 1

  def test_small_0_1_and_0_2_map_to_0_10(self):
    assert self.migrate(FM_LateralSmoothingEnabled=True, FM_LateralSmoothingSeconds=0.1).get("FM_SmallModelLatSmoothing") == 2
    assert self.migrate(FM_LateralSmoothingEnabled=True, FM_LateralSmoothingSeconds=0.2).get("FM_SmallModelLatSmoothing") == 2

  def test_small_on_unset_seconds_uses_old_default_0_1(self):
    assert self.migrate(FM_LateralSmoothingEnabled=True).get("FM_SmallModelLatSmoothing") == 2

  def test_big_off_and_on(self):
    assert self.migrate(FM_BigModelSmoothing=False).get("FM_BigModelLatSmoothing") == 0
    assert self.migrate(FM_BigModelSmoothing=True).get("FM_BigModelLatSmoothing") == 2

  def test_fresh_install_writes_nothing(self):
    p = self.migrate()
    assert p.get("FM_SmallModelLatSmoothing") is None
    assert p.get("FM_BigModelLatSmoothing") is None

  def test_old_keys_untouched_and_chosen_value_kept(self):
    params = Params()
    for k in self.KEYS:
      params.remove(k)
    params.put("FM_SmallModelLatSmoothing", 1, block=True)
    params.put("FM_BigModelLatSmoothing", 1, block=True)
    for k, v in {"FM_LateralSmoothingEnabled": True, "FM_LateralSmoothingSeconds": 0.2, "FM_BigModelSmoothing": True}.items():
      params.put(k, v, block=True)
    names = ("small_model_lat_smoothing", "big_model_lat_smoothing")
    assert run(params, [m for m in fm_migration.MIGRATIONS if m[0] in names]) == list(names)
    assert params.get("FM_SmallModelLatSmoothing") == 1 and params.get("FM_BigModelLatSmoothing") == 1
    assert params.get("FM_LateralSmoothingEnabled") is True
    assert params.get("FM_LateralSmoothingSeconds") == 0.2
    assert params.get("FM_BigModelSmoothing") is True
