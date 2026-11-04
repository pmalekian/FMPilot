"""Basic tests for tools/fm on small synthetic rlogs written with the fork's own cereal schema.

run: cd tools/fm && python -m pytest tests
"""
import contextlib
import io
import json
import os
import sys

import numpy as np
import pytest
import zstandard

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fmlog
import turns
from fmlog import log

ROUTE = "000001a3--c20ba54385"
SEG_S = 10.0  # short segments keep the tests fast
BUNDLE = {"internalName": "tsfdom", "displayName": "TSFDOM", "ref": "tsfdom-ref",
          "models": [{"artifact": {"fileName": "tsfdom_driving_tinygrad.pkl"}}]}


def settings(custom=False, base_ref="tsfdom-ref", enabled=True):
  p = {"ModelManager_ActiveBundle": json.dumps(BUNDLE).encode(), "GitCommit": b"a0dca062ab22f7db"}
  if custom:
    p |= {"FM_CustomModelPath": b"/data/fm/fm_opm16d_rlm_splice_tinygrad.pkl", "FM_CustomModelBaseRef": base_ref.encode(),
          "FM_CustomModelEnabled": b"1" if enabled else b"0"}
  return p


def curvature_at(t):
  """A right turn (curvature positive to the right) from 8 s to 13 s of the drive, across the --0/--1 boundary."""
  return 0.05 if 8.0 <= t < 13.0 else 0.0


def segment(n, prm, loaded=None, turn=curvature_at, hand=0.0, init_t=None):
  """Events of segment n: 100 Hz car/controls, 20 Hz model; drive time t = n * SEG_S + local time."""
  evs = []

  def ev(t, kind):
    e = log.Event.new_message()
    e.logMonoTime = int((1000.0 + t) * 1e9)
    e.valid = True
    return e, (e.init(kind) if kind != "logMessage" else None)

  e, init = ev(n * SEG_S if init_t is None else init_t, "initData")
  init.gitCommit = prm["GitCommit"].decode()
  entries = init.init("params").init("entries", len(prm))
  for x, (k, v) in zip(entries, prm.items(), strict=True):
    x.key, x.value = k, v
  evs.append(e)
  if loaded:
    e, _ = ev(n * SEG_S + 0.001, "logMessage")
    e.logMessage = json.dumps({"msg": f"loading combined pkl: /data/models/{loaded}"})
    evs.append(e)
  for i in range(int(SEG_S * 100)):
    t = n * SEG_S + i / 100 + 0.01
    k = turn(t)
    e, cs = ev(t, "carState")
    cs.vEgo, cs.steeringAngleDeg = 5.0, -k * 1500  # angle is left-positive: a right turn reads negative
    cs.steeringTorque = hand
    evs.append(e)
    e, ctl = ev(t, "controlsState")
    ctl.curvature, ctl.desiredCurvature = k, k
    ctl.lateralControlState.init("torqueState").output = -min(k * 20, 1.0)  # torque is left-positive
    evs.append(e)
    e, cc = ev(t, "carControl")
    cc.latActive = True
    cc.actuators.torque = -min(k * 20, 1.0)
    evs.append(e)
    e, co = ev(t, "carOutput")
    co.actuatorsOutput.torque = -min(k * 20, 1.0)
    evs.append(e)
    if i % 5 == 0:
      e, m = ev(t, "modelV2")
      m.action.desiredCurvature = k
      lines = m.init("laneLines", 4)
      for j, line in enumerate(lines):
        line.init("y", 33)
        line.y[0] = (-1.8, -1.8, 1.8, 1.8)[j]
      m.laneLineProbs = [0.9] * 4
      m.modelExecutionTime = 0.037
      evs.append(e)
  return evs


def write(path, evs, compress=True):
  data = b"".join(e.to_bytes() for e in evs)
  os.makedirs(os.path.dirname(path), exist_ok=True)
  with open(path, "wb") as f:
    f.write(zstandard.ZstdCompressor().compress(data) if compress else data)
  return str(path)


@pytest.fixture
def drive_dir(tmp_path):
  """Segments --0, --1 and --3 of one drive (--2 not downloaded)."""
  prm = settings()
  for n in (0, 1, 3):
    write(tmp_path / f"{ROUTE}--{n}" / "rlog.zst", segment(n, prm, loaded="tsfdom_driving_tinygrad.pkl" if n == 0 else None))
  return tmp_path


def run(fn, *args):
  out = io.StringIO()
  with contextlib.redirect_stdout(out):
    fn(*args)
  return out.getvalue()


class TestPaths:
  def test_route_segment(self):
    assert fmlog.route_segment(f"/data/media/0/realdata/{ROUTE}--12/rlog.zst") == (ROUTE, 12)
    assert fmlog.route_segment(f"C:\\Users\\frank\\logs\\{ROUTE}--3\\rlog.zst") == (ROUTE, 3)
    assert fmlog.route_segment("dongle|2026-11-03--09-12-33--4--rlog.zst") == ("2026-11-03--09-12-33", 4)
    assert fmlog.route_segment("rlog.zst") == (None, None)

  def test_expand_folder_and_wildcard(self, drive_dir):
    assert len(fmlog.expand([str(drive_dir)])) == 3
    assert len(fmlog.expand([str(drive_dir / f"{ROUTE}--*" / "rlog.zst")])) == 3  # PowerShell passes * through

  def test_plain_rlog(self, tmp_path):
    p = write(tmp_path / "x" / "rlog", segment(0, settings()), compress=False)
    assert any(e.which() == "carState" for e in fmlog.read(p))


class TestDrive:
  def test_joins_in_segment_order(self, drive_dir):
    paths = [str(drive_dir / f"{ROUTE}--{n}" / "rlog.zst") for n in (3, 0, 1)]
    (d,) = fmlog.load_drives(paths)
    assert d.nums == [0, 1, 3]
    assert d.label() == f"{ROUTE} --0..--3 (missing --2)"
    assert d.where(12.5).strip() == "--1 + 2.5s"
    t = [d.t(e) for e in d.evs]
    assert t == sorted(t)

  def test_labels_when_init_data_carries_the_drive_start(self, tmp_path):
    # on the device every segment's initData has the drive's start time (route 0000002d: --3 and --4 labelled as --4)
    prm = settings()
    paths = [write(tmp_path / f"{ROUTE}--{n}" / "rlog.zst", segment(n, prm, init_t=0.0)) for n in (0, 3, 4)]
    (d,) = fmlog.load_drives(paths)
    assert d.where(5.0).strip() == "--0 + 5.0s"
    assert d.where(32.0).strip() == "--3 + 2.0s"
    assert d.where(41.0).strip() == "--4 + 1.0s"

  def test_keep_reads_only_some_messages(self, drive_dir):
    p = str(drive_dir / f"{ROUTE}--0" / "rlog.zst")
    kept = fmlog.read(p, keep={"initData", "carState"})
    assert {e.which() for e in kept} == {"initData", "carState"}
    assert len([e for e in kept if e.which() == "carState"]) == len([e for e in fmlog.read(p) if e.which() == "carState"])

  def test_gap_has_no_segment_label(self, drive_dir):
    (d,) = fmlog.load_drives([str(drive_dir)])
    assert d.where(25.0).strip() == "25.0s"  # inside the missing --2
    assert d.where(31.0).strip() == "--3 + 1.0s"

  def test_missing_path_is_reported(self, tmp_path):
    with pytest.raises(SystemExit):
      fmlog.expand([str(tmp_path / "nope--*" / "rlog.zst")])

  def test_each_keeps_files_apart(self, drive_dir):
    assert len(fmlog.load_drives([str(drive_dir)], each=True)) == 3

  def test_split_runs_never_bridges_a_gap(self):
    t = np.array([0, 1, 2, 3, 100, 101, 102], float)
    runs = fmlog.split_runs(t, np.ones(7, bool))
    assert [list(r) for r in runs] == [[0, 1, 2, 3], [4, 5, 6]]


class TestModelName:
  def evs(self, prm, loaded=None):
    return segment(1, prm, loaded=loaded)[:2]

  def test_confirmed_by_load_line(self):
    assert fmlog.model_info(self.evs(settings(), "tsfdom_driving_tinygrad.pkl")) == ("TSFDOM", True)

  def test_later_segment_from_settings(self):
    assert fmlog.model_info(self.evs(settings())) == ("TSFDOM", False)
    assert fmlog.model_name(self.evs(settings())) == "TSFDOM (from settings)"

  def test_custom_model(self):
    assert fmlog.model_info(self.evs(settings(custom=True))) == ("fm_opm16d_rlm_splice", False)
    loaded = self.evs(settings(custom=True), "../fm/fm_opm16d_rlm_splice_tinygrad.pkl")
    assert fmlog.model_info(loaded) == ("fm_opm16d_rlm_splice", True)

  def test_custom_switched_off_or_other_base(self):
    assert fmlog.model_info(self.evs(settings(custom=True, enabled=False)))[0] == "TSFDOM"
    assert fmlog.model_info(self.evs(settings(custom=True, base_ref="other")))[0] == "TSFDOM"

  def test_custom_file_missing_falls_back(self):
    loaded = self.evs(settings(custom=True), "tsfdom_driving_tinygrad.pkl")
    assert fmlog.model_info(loaded) == ("TSFDOM", True)

  def test_unknown(self):
    assert fmlog.model_name(self.evs({"GitCommit": b"x"})) == "?"


class TestHands:
  def test_with_openpilot_is_the_driver(self):
    h = turns.hand_split([-2.0] * 10, [-0.5] * 10, [True] * 10, -1)  # right turn, pushing right with openpilot
    assert h["with_op"] == 100 and h["against_op"] == 0

  def test_against_openpilot_is_ambiguous(self):
    h = turns.hand_split([2.0] * 10, [-0.5] * 10, [True] * 10, -1)  # could be the wheel's reaction
    assert h["against_op"] == 100 and h["with_op"] == 0
    assert np.isnan(h["driver_into"])

  def test_driver_alone_into_the_turn(self):
    h = turns.hand_split([-2.0, -2.0, 0.0, 2.0], [0.0] * 4, [False] * 4, -1)
    assert h["driver_into"] == 50
    assert np.isnan(h["with_op"])

  def test_light_hands_dont_count(self):
    h = turns.hand_split([0.3] * 10, [-0.5] * 10, [True] * 10, -1)
    assert h["with_op"] == 0 and h["against_op"] == 0


class TestTools:
  def test_turn_across_segments_is_one_turn(self, drive_dir):
    (d,) = fmlog.load_drives([str(drive_dir)])
    out = run(turns.main, d)
    assert "model TSFDOM, 1 sharp turns" in out
    line = [x for x in out.splitlines() if "mph R" in x]
    assert len(line) == 1 and line[0].strip().startswith("--0 +")
    assert "driven r  20.0 m" in line[0]

  def test_split_segments_would_see_two(self, drive_dir):
    outs = [run(turns.main, d) for d in fmlog.load_drives([str(drive_dir)], each=True)]
    assert sum(o.count("mph R") for o in outs) == 2
    assert "TSFDOM (from settings)" in outs[1]

  def test_turn_not_bridged_across_missing_segment(self, tmp_path):
    k = lambda t: 0.05 if 15.0 <= t else 0.0  # noqa: E731  # turning at the end of --1 and all of --3
    prm = settings()
    for n in (1, 3):
      write(tmp_path / f"{ROUTE}--{n}" / "rlog.zst", segment(n, prm, turn=k))
    (d,) = fmlog.load_drives([str(tmp_path)])
    assert "2 sharp turns" in run(turns.main, d)

  def test_other_tools_run(self, drive_dir):
    import dm
    import events
    import sway
    import timeline
    (d,) = fmlog.load_drives([str(drive_dir)])
    a = fmlog.log_args("", **{"--step": {"type": float, "default": 2.0}, "--start": {"type": float, "default": 0.0},
                              "--end": {"type": float, "default": 1e9}}).parse_args(["x"])
    out = run(timeline.main, d, a)
    assert "no log for 10 s" in out and "--3 +" in out
    assert "TSFDOM" in run(events.main, d)
    assert "no straight engaged stretch" in run(sway.main, d, 20.0)  # 11 mph in the synthetic drive
    assert "no driver monitoring data" in run(dm.main, d)
