"""Diagnostic timing for hardwared's deviceState loop.

hardware_thread runs at 2 Hz, and a stall of a few seconds stops deviceState altogether, which selfdrived
reports as a communication issue ("Low Communication Rate Between Processes", or a disengage once it passes
5 s). LoopTimer times each phase of one iteration and the params calls inside it, and reports the breakdown
whenever an iteration runs long.

FM Pilot (Nov 16, 2026): drives 0000008e--6 and others showed 2.3 s and 4.2 s stalls whose time fell almost
entirely outside every timed phase. So the report now also carries:
  untimed_ms        work time no phase accounts for
  reads             each thermal-zone read, by zone name (thermal_config.get_msg() was untimed)
  watchdog_late_ms  how late a 20 Hz watchdog thread woke during the iteration. Near the stall's length means
                    the whole process stopped (a thread holding the GIL, or the process not being scheduled);
                    near zero means only the loop thread was blocked, in a call that releases the GIL
  stack             where the loop thread was once its work had run STACK_AFTER_S, taken by the watchdog
  threads           the other threads' current line at that moment
Kept free of openpilot imports so it can be tested on its own.
"""
import os
import sys
import threading
import time
import traceback
from contextlib import contextmanager

# Phases that block on purpose and must not count as time the device lost. sm.update() polls
# pandaStates, which publishes at 10 Hz, with a 1.5x timeout, so the roughly 100 ms it spends waiting
# is how this loop paces itself. Judging the threshold on the raw total counted that wait as work and
# fired on nearly every iteration: 4268 reports in one 14 minute drive, 3.4 MB of undecimated log, of
# which 7 were real. These phases are still reported, because a publisher in trouble shows up as one
# of them repeatedly hitting its timeout, but they are left out of the "was this slow" decision.
BLOCKING_PHASES = ("sm_update",)

WATCHDOG_PERIOD_S = 0.05
STACK_AFTER_S = 0.3   # well past a normal iteration's work (~5-20 ms), well short of the stalls seen (1-4 s)
STACK_DEPTH = 8


def _where(frame) -> str:
  return f"{os.path.basename(frame.f_code.co_filename)}:{frame.f_lineno} {frame.f_code.co_name}"


def _stack(frame) -> list[str]:
  return [f"{os.path.basename(fs.filename)}:{fs.lineno} {fs.name}" for fs in traceback.extract_stack(frame)[-STACK_DEPTH:]]


class LoopTimer:
  """Collects per phase timings for a single hardware_thread iteration and reports slow ones.

  The cost is two clock reads per measurement, so this stays enabled while driving. A report is only
  emitted when an iteration spends more than slow_ms doing actual work, so a healthy device stays
  silent. "Actual work" is the whole iteration minus BLOCKING_PHASES, the waits this loop takes
  deliberately; see the note there for why the difference matters.
  """

  def __init__(self, slow_ms: float):
    self.slow_ms = slow_ms
    self.started = 0.
    self.phases: dict[str, float] = {}
    self.params: dict[str, float] = {}
    self.reads: dict[str, float] = {}
    # watchdog state, written by the watchdog thread and reset by start()
    self.loop_ident: int | None = None
    self.work_since: float | None = None  # None while inside a blocking phase
    self.watchdog_late_ms = 0.
    self.stack: list[str] | None = None
    self.threads: dict[str, str] | None = None
    self.stack_at_ms = 0.

  def start(self) -> None:
    self.phases = {}
    self.params = {}
    self.reads = {}
    self.watchdog_late_ms = 0.
    self.stack = None
    self.threads = None
    self.stack_at_ms = 0.
    self.started = time.monotonic()
    self.work_since = self.started

  def record(self, name: str, ms: float) -> None:
    # summed, so a key touched twice in one iteration still reports its full cost
    self.params[name] = self.params.get(name, 0.) + ms

  def record_read(self, name: str, ms: float) -> None:
    self.reads[name] = self.reads.get(name, 0.) + ms

  @contextmanager
  def phase(self, name: str):
    blocking = name in BLOCKING_PHASES
    if blocking:
      self.work_since = None
    t0 = time.monotonic()
    try:
      yield
    finally:
      t1 = time.monotonic()
      self.phases[name] = self.phases.get(name, 0.) + (t1 - t0) * 1e3
      if blocking:
        self.work_since = t1

  # ---- watchdog ----

  def start_watchdog(self, end_event: threading.Event | None = None) -> threading.Thread:
    """Watch the calling thread. Must be called from the loop thread."""
    self.loop_ident = threading.get_ident()
    t = threading.Thread(target=self._watch, args=(end_event,), name="hw_loop_watchdog", daemon=True)
    t.start()
    return t

  def _watch(self, end_event: threading.Event | None) -> None:
    last = time.monotonic()
    while end_event is None or not end_event.is_set():
      time.sleep(WATCHDOG_PERIOD_S)
      now = time.monotonic()
      try:
        self.check(now, (now - last - WATCHDOG_PERIOD_S) * 1e3)
      except Exception:
        pass  # a diagnostic must never take hardwared down
      last = now

  def check(self, now: float, late_ms: float) -> None:
    """One watchdog tick: note lateness, and take the loop thread's stack once its work runs long."""
    self.watchdog_late_ms = max(self.watchdog_late_ms, late_ms)
    since = self.work_since
    if since is None or self.stack is not None or self.loop_ident is None or now - since < STACK_AFTER_S:
      return
    frames = sys._current_frames()
    frame = frames.get(self.loop_ident)
    if frame is None:
      return
    names = {t.ident: t.name for t in threading.enumerate()}
    me = threading.get_ident()
    self.threads = {names.get(i, str(i)): _where(f) for i, f in frames.items() if i not in (self.loop_ident, me)}
    self.stack_at_ms = round((now - self.started) * 1e3, 1)
    self.stack = _stack(frame)

  # ---- report ----

  def report(self) -> dict | None:
    """The slow-loop event's fields, or None when the iteration was not slow."""
    total_ms = (time.monotonic() - self.started) * 1e3
    blocked_ms = sum(self.phases.get(n, 0.) for n in BLOCKING_PHASES)
    work_ms = total_ms - blocked_ms
    if work_ms < self.slow_ms:
      return None
    timed_ms = sum(ms for n, ms in self.phases.items() if n not in BLOCKING_PHASES)
    untimed_ms = max(work_ms - timed_ms, 0.)
    # params and reads are nested inside the phase timings (or in untimed code), so each is reported separately
    slow_phases = {n: round(ms, 1) for n, ms in self.phases.items() if ms >= 1.}
    slow_params = {n: round(ms, 1) for n, ms in self.params.items() if ms >= 1.}
    slow_reads = {n: round(ms, 1) for n, ms in self.reads.items() if ms >= 1.}
    # a deliberate wait can never be the culprit this report exists to name, so it is not a candidate
    # for "worst" even though it stays in phases for anyone reading the whole picture
    candidates = [kv for kv in list(slow_phases.items()) + list(slow_params.items()) + list(slow_reads.items())
                  if kv[0] not in BLOCKING_PHASES]
    candidates.append(("untimed", round(untimed_ms, 1)))
    worst = max(candidates, key=lambda kv: kv[1])
    ev = {"work_ms": round(work_ms, 1), "total_ms": round(total_ms, 1), "blocked_ms": round(blocked_ms, 1),
          "untimed_ms": round(untimed_ms, 1), "worst": worst[0], "worst_ms": worst[1], "phases": slow_phases,
          "params": slow_params, "reads": slow_reads, "watchdog_late_ms": round(self.watchdog_late_ms, 1)}
    if self.stack is not None:
      ev.update(stack=self.stack, stack_at_ms=self.stack_at_ms, threads=self.threads)
    return ev
