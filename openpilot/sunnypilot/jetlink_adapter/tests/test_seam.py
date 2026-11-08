"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

FM Pilot: zoompilot develop 92e1ecb's test_seam.py (MIT) with AdventurePilot PR #12's adapter, fallback and catalog
checks (Nimijneb, MIT) added at the end.

Where jetlink meets modeld, in both modelds: stock modeld and sunnypilot's
modeld_tinygrad, which a custom small bundle runs on.

The hook lines are jetlink's check (testing.check_seam): prepare() before
modeld goes realtime, attach() once the small model is built, its result run,
handed_over() and state() read. And the joining model is held to whatever the
loop reads and writes on the model it runs: a read it cannot answer is found
here rather than on the frame thread of a drive. Both read the modelds rather
than import them (that costs tinygrad, usb1 and a vision stream).
"""
import ast
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from jetlink.openpilot.joining import JoiningModelState
from jetlink.openpilot.testing import check_adapter, check_fallback, check_seam

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.jetlink_adapter.tests.support import GuardedTest

OPENPILOT = Path(__file__).resolve().parents[3]


def _main(src: str) -> ast.FunctionDef:
  return next(n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == 'main')


def _model_attributes(src: str) -> tuple[set[str], set[str]]:
  """What main() reads and writes on `model`, the model the loop runs: every
  `model.<attr>` load and store, and getattr(model, '<attr>', ...)."""
  loads, stores = set(), set()
  for n in ast.walk(_main(src)):
    if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) and n.value.id == 'model':
      (stores if isinstance(n.ctx, ast.Store) else loads).add(n.attr)
    elif (isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'getattr' and len(n.args) >= 2
          and isinstance(n.args[0], ast.Name) and n.args[0].id == 'model' and isinstance(n.args[1], ast.Constant)):
      loads.add(n.args[1].value)
  return loads, stores


class JoiningContract:
  PATH: Path

  def test_the_hooks_are_where_jetlink_needs_them(self):
    self.assertEqual(check_seam(self.PATH), [])

  def test_the_joining_model_answers_every_read_and_takes_every_write(self):
    # read out of main() rather than kept by hand. A read the joining model
    # cannot answer is an AttributeError on the frame thread, which modeld
    # re-raises: no driving model for the drive, on jetlink devices only. A
    # write without a setter lands on the joining model as a plain attribute,
    # which it never reads
    loads, stores = _model_attributes(self.PATH.read_text())
    self.assertIn('run', loads, "this no longer finds what the loop reads off the model")
    self.assertEqual(sorted(n for n in loads | stores if n.startswith('_')), [])
    for name in stores:
      prop = getattr(JoiningModelState, name, None)
      self.assertTrue(isinstance(prop, property) and prop.fset is not None,
                      f"{self.PATH.name} writes model.{name}, which JoiningModelState keeps to itself")

    small = SimpleNamespace(**dict.fromkeys(loads, 0.0))
    stop = threading.Event()

    def connect(should_stop=None):
      stop.wait(5)
      raise ConnectionError("no link in this test")

    joining = JoiningModelState(small, connect, build=None, progress=mock.MagicMock(), log=mock.MagicMock())
    try:
      missing = []
      for name in sorted(loads):
        try:
          getattr(joining, name)
        except AttributeError:
          missing.append(name)
      self.assertEqual(missing, [], f"JoiningModelState cannot answer {missing}, which {self.PATH.name} reads")
    finally:
      stop.set()
      joining.close()


class StockModeld(JoiningContract, OpenpilotTestCase):
  PATH = OPENPILOT / 'selfdrive' / 'modeld' / 'modeld.py'


class ModeldTinygrad(JoiningContract, OpenpilotTestCase):
  PATH = OPENPILOT / 'sunnypilot' / 'modeld_v2' / 'modeld.py'



class TestJetlinkAdapter(GuardedTest, OpenpilotTestCase):
  def test_the_adapter_conforms(self):
    heavy = ("numpy", "capnp", "zmq", "cereal", "msgq", "tinygrad", "openpilot.cereal",
             "openpilot.common.params", "openpilot.common.swaglog")
    assert check_adapter(jetlink_adapter, heavy=heavy) == []

  def test_every_hook_answers_as_the_link_off(self):
    assert check_fallback(jetlink_adapter._Off("why"), jetlink_adapter.HOOK_NAMES,
                          link_on=jetlink_adapter.link_on) == []

  def test_the_model_manager_keeps_its_own_big_model_list(self):
    """FM Pilot doesn't call the model-list hooks (lists_big_models, big_catalog, listed_ref): the big model is picked
    on the Jetlink computer or in the comma's chestnut slot, and sunnypilot's list stays the only one fetched here."""
    fetcher = OPENPILOT / "sunnypilot/models/fetcher.py"
    source = fetcher.read_text()
    for hook in ("extend_catalog", "big_catalog", "lists_big_models", "jetlink_adapter."):
      assert hook not in source, (fetcher, hook)


if __name__ == '__main__':
  unittest.main()
