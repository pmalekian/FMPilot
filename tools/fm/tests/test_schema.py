"""The log tools read both layouts: fm-dev-nav-settle and earlier (FM fields on sunnypilot's structs, tools/fm's schema
copy) and fm-dev-sync (FM's own messages, the repo's schema), picked per log file.

run: cd tools/fm && python -m pytest tests
"""
import os
import sys

import pytest
import zstandard

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fmlog
import settle
import turns

ROUTE = "000001b0--d3a1b2c3d4"


def event(schema, t, kind):
  e = schema.Event.new_message()
  e.logMonoTime = int((1000.0 + t) * 1e9)
  e.valid = True
  return e, e.init(kind)


def new_layout(branch="fm-dev-sync", with_init=True, with_fm=True):
  """A few seconds of an fm-dev-sync drive: a right turn with Turn Assist, Release Guard, the torque cap and a Settle
  After Turn push window."""
  L, evs = fmlog.log_new, []
  if with_init:
    e, init = event(L, 0.0, "initData")
    init.gitBranch = branch
    evs.append(e)
  for i in range(300):
    t = i * 0.01
    e, cs = event(L, t, "carState")
    cs.vEgo = 5.0
    evs.append(e)
    if not with_fm:
      continue
    e, c = event(L, t, "fmControls")
    c.turnAssist.sign = 1
    c.turnAssist.requestIn = 0.02
    c.turnAssist.requestOut = 0.05
    c.turnAssist.lead = 0.03
    c.releaseGuard.active = t > 1.0
    c.releaseGuard.cut = -0.2
    c.releaseGuard.limit = 0.5
    evs.append(e)
    e, c = event(L, t, "fmCarState")
    c.steerCap = 580
    evs.append(e)
    if i % 5 == 0:
      e, m = event(L, t, "fmModel")
      if 0.5 <= t < 2.0:
        m.turnSettle = 0.002
        m.turnSettlePhase = "push"
        m.turnSettlePush = 0.1 + 0.1 * t
        m.turnSettleLimit = "cap" if t > 1.8 else "none"
      evs.append(e)
  return evs


def old_layout(branch="fm-dev-nav-settle"):
  L, evs = fmlog.log_old, []
  e, init = event(L, 0.0, "initData")
  init.gitBranch = branch
  evs.append(e)
  for i in range(50):
    t = i * 0.01
    e, c = event(L, t, "carControlSP")
    c.turnAssist.sign = 1
    c.turnAssist.requestOut = 0.05
    c.turnAssist.boost = 0.01
    evs.append(e)
    e, c = event(L, t, "carStateSP")
    c.fmSteerCap = 385
    evs.append(e)
  return evs


def raw(evs):
  return b"".join(e.to_bytes() for e in evs)


def write(path, evs):
  os.makedirs(os.path.dirname(path), exist_ok=True)
  with open(path, "wb") as f:
    f.write(zstandard.ZstdCompressor().compress(raw(evs)))
  return str(path)


class TestPickSchema:
  def test_fm_dev_sync_branch_is_new(self):
    assert fmlog.pick_schema(raw(new_layout(with_fm=False))) == "new"

  def test_pr_branch_is_new(self):
    assert fmlog.pick_schema(raw(new_layout(branch="fm-sync/pr04-torque", with_fm=False))) == "new"

  def test_fm_message_is_new_whatever_the_branch(self):
    assert fmlog.pick_schema(raw(new_layout(branch="my-fork", with_init=True))) == "new"
    assert fmlog.pick_schema(raw(new_layout(with_init=False))) == "new"

  def test_old_branches_are_old(self):
    assert fmlog.pick_schema(raw(old_layout())) == "old"
    assert fmlog.pick_schema(raw(old_layout(branch="fm-dev-max"))) == "old"
    assert fmlog.pick_schema(raw(new_layout(branch="dev-next-src", with_fm=False))) == "old"  # AP: no FM data at all

  def test_override(self, monkeypatch):
    monkeypatch.setenv("FM_SCHEMA", "old")
    assert fmlog.pick_schema(raw(new_layout())) == "old"
    monkeypatch.setenv("FM_SCHEMA", "new")
    assert fmlog.pick_schema(raw(old_layout())) == "new"


class TestParts:
  def test_same_values_from_both_layouts(self, tmp_path):
    new = fmlog.read(write(tmp_path / f"{ROUTE}--0" / "rlog.zst", new_layout()))
    old = fmlog.read(write(tmp_path / f"{ROUTE[:-1]}5--0" / "rlog.zst", old_layout()))
    ta_new = next(p for e in new if (p := fmlog.fm_part(e, "turnAssist")) is not None)
    ta_old = next(p for e in old if (p := fmlog.fm_part(e, "turnAssist")) is not None)
    assert ta_new.sign == ta_old.sign == 1
    assert ta_new.requestOut == pytest.approx(ta_old.requestOut)
    assert {fmlog.fm_part(e, "steerCap") for e in new} - {None} == {580}
    assert {fmlog.fm_part(e, "steerCap") for e in old} - {None} == {385}

  def test_other_layouts_message_gives_none(self):
    e, _ = event(fmlog.log_new, 0.0, "carControlSP")  # the new carControlSP has no turnAssist
    assert fmlog.fm_part(e.as_reader(), "turnAssist") is None
    e, _ = event(fmlog.log_new, 0.0, "modelDataV2SP")  # nor Settle's fields
    assert fmlog.fm_part(e.as_reader(), "settle") is None


class TestToolsOnNewLayout:
  def test_turn_assist_series(self, tmp_path):
    write(tmp_path / f"{ROUTE}--0" / "rlog.zst", new_layout())
    (d,) = fmlog.load_drives([str(tmp_path)])
    ta = turns.assist_series(d)
    assert ta is not None and ta["lead"].max() == pytest.approx(0.03)
    assert ta["boost"].max() == 0.0  # Turn Boost was dropped

  def test_torque_and_guard_series(self, tmp_path):
    write(tmp_path / f"{ROUTE}--0" / "rlog.zst", new_layout())
    (d,) = fmlog.load_drives([str(tmp_path)])
    *_, sp, rg = turns.series(d)
    assert sp[:, 1].max() == 580
    assert rg[:, 1].max() == 1.0 and rg[:, 2].max() == pytest.approx(0.2)

  def test_settle_windows(self, tmp_path):
    write(tmp_path / f"{ROUTE}--0" / "rlog.zst", new_layout())
    (d,) = fmlog.load_drives([str(tmp_path)], keep={"carState", "modelDataV2SP", "fmModel"})
    (w,) = settle.windows(d.evs, d.evs[0].logMonoTime)
    assert w.phases == ["push"] and "cap" in w.limits
    assert w.push == pytest.approx(0.1 + 0.1 * 1.95, abs=0.01)
