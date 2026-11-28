"""FM Pilot: WifiManager keeps trying a system D-Bus that is slow at boot instead of giving up until a reboot."""
import time

from openpilot.common.test import Mocker, OpenpilotTestCase
from openpilot.system.ui.lib import wifi_manager
from openpilot.system.ui.lib.wifi_manager import WifiManager


def _patch(mocker: Mocker, failures: int, pc: bool = False):
  calls = {"n": 0}

  def open_threading(bus):
    calls["n"] += 1
    if calls["n"] <= failures:
      raise TimeoutError("D-Bus too slow")
    return mocker.MagicMock()

  mocker.patch.object(wifi_manager, "open_dbus_connection_threading", side_effect=open_threading)
  mocker.patch.object(wifi_manager, "open_dbus_connection_blocking", return_value=mocker.MagicMock())
  mocker.patch.object(wifi_manager, "DBusRouter", side_effect=lambda conn: mocker.MagicMock(conn=conn))
  mocker.patch.object(wifi_manager, "DBUS_RETRY_SECONDS", (0.01,))
  mocker.patch.object(wifi_manager, "PC", pc)
  mocker.patch.object(wifi_manager, "Params", None)
  initialize = mocker.patch.object(WifiManager, "_initialize")
  return calls, initialize


def _wait(cond, timeout=2.0):
  end = time.monotonic() + timeout
  while time.monotonic() < end and not cond():
    time.sleep(0.01)
  return cond()


class TestDbusRetry(OpenpilotTestCase):
  def test_connects_first_time(self, mocker):
    calls, initialize = _patch(mocker, failures=0)
    wm = WifiManager()
    self.assertFalse(wm._exit)
    initialize.assert_called_once()
    self.assertEqual(calls["n"], 1)
    wm._stopped = True

  def test_slow_dbus_is_retried_then_initializes(self, mocker):
    calls, initialize = _patch(mocker, failures=3)
    wm = WifiManager()
    self.assertTrue(wm._exit)  # nothing runs while D-Bus is unreachable
    self.assertTrue(_wait(lambda: initialize.call_count == 1), "never initialized after D-Bus came up")
    self.assertFalse(wm._exit)
    self.assertIsNotNone(wm._router_main)
    self.assertIsNotNone(wm._conn_monitor)
    self.assertEqual(calls["n"], 4)
    time.sleep(0.05)
    self.assertEqual(calls["n"], 4, "kept retrying after connecting")
    wm._stopped = True

  def test_stop_ends_the_retry(self, mocker):
    calls, initialize = _patch(mocker, failures=10**9)
    wm = WifiManager()
    self.assertTrue(_wait(lambda: calls["n"] >= 3))
    wm.stop()
    time.sleep(0.05)
    n = calls["n"]
    time.sleep(0.05)
    self.assertLessEqual(calls["n"], n + 1)
    initialize.assert_not_called()

  def test_pc_without_dbus_gives_up_as_before(self, mocker):
    calls, initialize = _patch(mocker, failures=10**9, pc=True)
    wm = WifiManager()
    time.sleep(0.05)
    self.assertEqual(calls["n"], 1)
    self.assertTrue(wm._exit)
    initialize.assert_not_called()
