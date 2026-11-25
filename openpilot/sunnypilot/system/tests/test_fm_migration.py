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
