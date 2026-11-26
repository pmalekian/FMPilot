"""
The fork's side of jetlink: the modeld seams and the no-checkout fallback.
"""
import inspect
import re
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from jetlink.openpilot.testing import check_adapter, check_fallback, check_seam

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.jetlink_adapter.tests.support import GuardedTest

REPO = Path(__file__).resolve().parents[4]


class TestJetlinkSeam(GuardedTest, OpenpilotTestCase):
  def test_the_adapter_conforms(self):
    heavy = ("numpy", "capnp", "zmq", "cereal", "msgq", "tinygrad", "openpilot.cereal",
             "openpilot.common.params", "openpilot.common.swaglog")
    assert check_adapter(jetlink_adapter, heavy=heavy) == []

  def test_every_modeld_has_the_seam(self):
    for modeld in ("openpilot/selfdrive/modeld/modeld.py", "openpilot/sunnypilot/modeld_v2/modeld.py"):
      assert check_seam(str(REPO / modeld)) == [], modeld

  def test_the_large_model_takes_modeld_v2s_action_call(self):
    """Forward the floor and run the modeld_v2 lane-keeping callback on the stock action."""
    from openpilot.sunnypilot.modeld_v2.modeld import ModelState
    parameters = list(inspect.signature(ModelState.get_action_from_model).parameters.values())[1:]
    args = [object() for param in parameters if param.kind == inspect.Parameter.POSITIONAL_OR_KEYWORD]
    blendful = mock.Mock(return_value=0.002)
    args[4] = 10.0
    args[5] = blendful
    kwargs = {param.name: 2.5 for param in parameters if param.kind == inspect.Parameter.KEYWORD_ONLY}
    with mock.patch('openpilot.selfdrive.modeld.modeld.get_action_from_model',
                    return_value=SimpleNamespace(desiredCurvature=0.0)) as stock:
      action = jetlink_adapter.adapter().action_function()
      assert action(*args, **kwargs).desiredCurvature == 0.002
    stock.assert_called_once_with(*args[:5], action_speed_floor=2.5)
    blendful.assert_called_once_with(0.0)

  def test_every_hook_answers_as_the_link_off(self):
    assert check_fallback(jetlink_adapter._Off("why"), jetlink_adapter.HOOK_NAMES,
                          link_on=jetlink_adapter.link_on) == []
    off = jetlink_adapter._Off()
    catalog = {"bundles": []}
    assert not off.shutdown_pending() and not off.request_shutdown("test")
    assert not off.should_extend_catalog() and off.extend_catalog(catalog) is catalog

  def test_the_model_manager_keeps_its_own_big_model_list(self):
    """Its big_catalog() hook would make jetlink's unpinned model index the list
    the user picks from. Nothing outside the adapter calls the list hooks, so the
    list stays the catalog the model manager fetched from sunnypilot; keep it that
    way unless we choose otherwise.
    """
    catalog_hooks = "big_catalog|lists_big_models|listed_ref|should_extend_catalog|extend_catalog"
    hooks = re.compile(rf"\b(?:jetlink_adapter|hooks|HOOKS)\.(?:{catalog_hooks})\b")
    adapter = REPO / "openpilot/sunnypilot/jetlink_adapter"
    callers = [path for path in (REPO / "openpilot").rglob("*.py")
               if adapter not in path.parents and hooks.search(path.read_text(errors="ignore"))]
    assert callers == []
    fetcher = (REPO / "openpilot/sunnypilot/models/fetcher.py").read_text()
    assert "big_catalog" not in fetcher
    assert "lists_big_models" not in fetcher
    assert "listed_ref" not in fetcher
    assert "jetlink_adapter." not in fetcher
