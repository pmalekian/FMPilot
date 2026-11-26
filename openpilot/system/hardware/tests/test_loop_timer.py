import threading
import time

from openpilot.system.hardware import loop_timer as lt_mod
from openpilot.system.hardware.loop_timer import LoopTimer


def _timer(slow_ms=100.):
  t = LoopTimer(slow_ms)
  t.start()
  return t


def test_fast_iteration_is_silent():
  t = _timer()
  with t.phase("device_stats"):
    pass
  assert t.report() is None


def test_blocking_phase_is_not_work():
  t = _timer(50.)
  with t.phase("sm_update"):
    time.sleep(0.12)
  assert t.report() is None


def test_untimed_stall_is_named():
  t = _timer(50.)
  with t.phase("device_stats"):
    pass
  time.sleep(0.12)  # outside every phase, like the Nov 16 stalls
  ev = t.report()
  assert ev is not None
  assert ev["worst"] == "untimed"
  assert ev["untimed_ms"] >= 100.
  assert ev["untimed_ms"] <= ev["work_ms"]


def test_untimed_excludes_timed_phases_and_blocking():
  t = _timer(50.)
  with t.phase("sm_update"):
    time.sleep(0.05)
  with t.phase("kmsg"):
    time.sleep(0.08)
  ev = t.report()
  assert ev["worst"] == "kmsg"
  assert ev["untimed_ms"] < 20.
  assert "sm_update" in ev["phases"] and ev["blocked_ms"] >= 45.


def test_slow_thermal_read_is_named():
  t = _timer(50.)
  with t.phase("thermal"):
    t.record_read("gnss", 90.)
    t.record_read("cpu0-silver-usr", 0.2)
    time.sleep(0.09)
  ev = t.report()
  assert ev["reads"] == {"gnss": 90.}
  assert ev["worst"] in ("gnss", "thermal")


def test_watchdog_lateness_is_kept_per_iteration():
  t = _timer()
  t.check(time.monotonic(), 5.)
  t.check(time.monotonic(), 2100.)
  t.check(time.monotonic(), 3.)
  assert t.watchdog_late_ms == 2100.
  t.start()
  assert t.watchdog_late_ms == 0.


def test_no_stack_inside_blocking_phase():
  t = _timer()
  t.loop_ident = threading.get_ident()
  with t.phase("sm_update"):
    t.check(time.monotonic() + 10., 0.)
  assert t.stack is None


def test_no_stack_before_threshold():
  t = _timer()
  t.loop_ident = threading.get_ident()
  t.check(t.started + lt_mod.STACK_AFTER_S / 2, 0.)
  assert t.stack is None


def _stuck_in_a_read(seconds):
  time.sleep(seconds)  # releases the GIL, like a blocked sysfs read


def test_watchdog_captures_the_blocked_line():
  t = LoopTimer(100.)
  end = threading.Event()
  t.start()
  t.start_watchdog(end)
  try:
    _stuck_in_a_read(lt_mod.STACK_AFTER_S + 0.25)
    ev = t.report()
  finally:
    end.set()
  assert ev is not None
  assert any("_stuck_in_a_read" in s for s in ev["stack"])
  assert ev["stack_at_ms"] >= lt_mod.STACK_AFTER_S * 1e3
  assert "hw_loop_watchdog" not in ev["threads"]
  # the watchdog ran on time: only the loop thread was blocked
  assert ev["watchdog_late_ms"] < 200.


def test_stack_taken_once_per_iteration():
  t = _timer()
  t.loop_ident = threading.get_ident()
  t.check(t.started + 1., 0.)
  first = t.stack
  t.check(t.started + 2., 0.)
  assert t.stack is first
  t.start()
  assert t.stack is None
