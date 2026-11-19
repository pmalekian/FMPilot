"""Tests for the Jetlink log report: the status timeline, the big-model name, and the Mac server.log reader.

run: cd tools/fm && python -m pytest tests
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fmlog
import jetlink
import maclog
from fmlog import log
from test_fm_tools import settings, segment


def status(t, big, name="", state="active", **kw):
  e = log.Event.new_message()
  e.logMonoTime = int((1000.0 + t) * 1e9)
  m = e.init("fmJetlinkStatus")
  m.bigDriving, m.drivingName, m.state = big, name, state
  for k, v in kw.items():
    setattr(m, k, v)
  return e.as_reader()


def model(t, big, ms=0.03):
  e = log.Event.new_message()
  e.logMonoTime = int((1000.0 + t) * 1e9)
  m = e.init("modelV2")
  m.big, m.modelExecutionTime = big, ms
  return e.as_reader()


def drive(*extra):
  evs = segment(0, settings())[:1]
  return sorted(evs + list(extra), key=lambda e: e.logMonoTime)


class TestStatus:
  def test_runs_and_swap(self):
    evs = drive(status(1, False, state="idle"), status(2, True, "Lebowski"), status(3, True, "Lebowski"),
                status(4, True, "Fairy tale"), status(5, False, state="idle"))
    t0 = evs[0].logMonoTime
    runs = jetlink.driving_runs(fmlog.jetlink_status(evs), lambda m: (m - t0) / 1e9)
    assert [(r[2], r[3]) for r in runs] == [(False, "small model"), (True, "Lebowski"), (True, "Fairy tale"), (False, "small model")]
    assert fmlog.big_models(evs) == ["Lebowski", "Fairy tale"]

  def test_changes_reported(self):
    evs = drive(status(1, True, "A", serverPickName="A"), status(2, True, "A", serverPickName="B"))
    ch = jetlink.changes(fmlog.jetlink_status(evs), lambda m: 0.0)
    assert ch == [(0.0, "serverPickName: 'A' -> 'B'")]

  def test_hash_prefix_when_unnamed(self):
    evs = drive(status(1, True, drivingSha256="abcdef0123456789"))
    assert fmlog.big_models(evs) == ["abcdef01"]


class TestModelName:
  def test_names_the_big_model(self):
    evs = drive(status(1, True, "Fairy tale"), model(1, True))
    assert fmlog.model_name(evs) == "Fairy tale (Jetlink big model)"

  def test_mixed_frames(self):
    evs = drive(status(1, True, "Fairy tale"), model(1, True), model(2, False))
    assert "50% of frames" in fmlog.model_name(evs) and "TSFDOM" in fmlog.model_name(evs)

  def test_old_log_with_big_frames_is_not_named_from_settings(self):
    assert fmlog.model_info(drive(model(1, True)))[0].startswith("Jetlink big model, not named")

  def test_small_model_unchanged(self):
    assert fmlog.model_info(drive(model(1, False))) == ("TSFDOM", False)

  def test_big_flag_read_from_modelv2_not_drivingmodeldata(self):
    assert fmlog.big_frames(drive(model(1, True), model(2, True))) == (2, 2)


class TestMacLog:
  TEXT = ("2026-11-19 08:40:12,163 INFO    jetlink.server: picked here: Fairy tale model\n"
          "2026-11-19 08:40:13,000 ERROR   jetlink.server: build failed\n"
          "Traceback (most recent call last):\n"
          "  boom\n"
          "2026-11-19 08:40:14,500 INFO    jetlink.server: hello from modeld/ac52ff69\n")

  def test_parse_joins_continuation_and_times(self):
    tz = maclog.zone("UTC")
    lines = maclog.parse(self.TEXT, tz)
    assert len(lines) == 3 and "boom" in lines[1][2]
    assert abs((lines[2][0] - lines[0][0]) - 2.337) < 1e-6
    assert lines[0][0] == 1795077612.163

  def test_skew_from_hello(self):
    lines = maclog.parse(self.TEXT, maclog.zone("UTC"))
    hello = lines[2][0]
    assert abs(maclog.skew([hello - 0.4], lines) - 0.4) < 1e-6
    assert maclog.skew([], lines) is None

  def test_window_and_filter(self):
    lines = maclog.parse(self.TEXT, maclog.zone("UTC"))
    t = lines[2][0]
    assert len(maclog.window(lines, t, t, before=2.0, after=0)) == 2
    assert maclog.keep_line("ERROR", "x") and maclog.keep_line("INFO", "picked here: x") and not maclog.keep_line("INFO", "frame 1")

  def test_wall_offset(self):
    evs = []
    for t in (1.0, 2.0, 3.0):
      e = log.Event.new_message()
      e.logMonoTime = int(t * 1e9)
      e.init("clocks").wallTimeNanos = int((1.7e9 + t + 5) * 1e9)
      evs.append(e.as_reader())
    assert abs(maclog.wall_offset(evs) - (1.7e9 + 5)) < 1e-3
    assert maclog.wall_offset([]) is None
