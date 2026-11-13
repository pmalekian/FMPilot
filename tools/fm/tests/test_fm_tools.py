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


def segment(n, prm, loaded=None, turn=curvature_at, hand=0.0, init_t=None, cap=None, can=None):
  """Events of segment n: 100 Hz car/controls, 20 Hz model; drive time t = n * SEG_S + local time. cap(t) and can(t),
  when given, log carStateSP.fmSteerCap and carOutput.actuatorsOutput.torqueOutputCan (counts)."""
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
    cs.vEgo, cs.vEgoRaw, cs.steeringAngleDeg = 5.0, 5.0, -k * 1500  # angle is left-positive: a right turn reads negative
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
    if can is not None:
      co.actuatorsOutput.torqueOutputCan = can(t)
    evs.append(e)
    if cap is not None:
      e, sp = ev(t, "carStateSP")
      sp.fmSteerCap = cap(t)
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


class TestTorqueLimits:
  @staticmethod
  def q(asked, sent, limit, lat=True):
    n = len(asked)
    return turns.torque_limits(asked, sent, np.broadcast_to(limit, n), np.broadcast_to(lat, n))

  def test_climbing_then_at_the_limit(self):
    n = 400
    sent = -np.minimum(580.0, 5.0 * np.arange(n))  # 5 per message: 116 messages to full
    r = self.q(np.full(n, -580.0), sent, 580)
    assert r["at_limit"] == pytest.approx(71, abs=2) and r["climbing"] == pytest.approx(29, abs=2)
    assert r["held"] == 0 and r["unwinding"] == 0 and r["peak"] == 580 and r["limit"] == 580

  def test_the_limit_in_force_is_what_counts(self):
    sent = np.full(100, 551.0)  # past 90 degrees the cap is 95%: asked 580, limit 551
    assert self.q(np.full(100, 580.0), sent, 551)["at_limit"] == 100
    assert self.q(np.full(100, 580.0), sent, 580)["at_limit"] == 0  # same torque, higher limit: not at it

  def test_unwinding_when_the_request_flips(self):
    sent = np.minimum(300.0, -300.0 + 5.0 * np.arange(200))  # still pushing -300 down to 0 while asked +300
    r = self.q(np.full(200, 300.0), sent, 580)
    assert r["unwinding"] == pytest.approx(28, abs=2) and r["at_limit"] == 0  # 56 of 200 messages are still above 20

  def test_held_below_the_request_without_rising(self):
    r = self.q(np.full(100, 300.0), np.full(100, 100.0), 580)
    assert r["held"] == 100 and r["climbing"] == 0

  def test_follows_the_request_is_none_of_them(self):
    r = self.q(np.full(100, 200.0), np.full(100, 200.0), 580)
    assert (r["at_limit"], r["climbing"], r["held"], r["unwinding"]) == (0, 0, 0, 0)

  def test_only_openpilot_steered_frames_count(self):
    lat = np.arange(100) >= 50
    r = self.q(np.full(100, 580.0), np.full(100, 580.0), 580, lat)
    assert r["at_limit"] == 100
    assert np.isnan(self.q(np.full(10, 580.0), np.zeros(10), 580, False)["at_limit"])

  def test_speed_curve_matches_the_car_code(self):
    values = pytest.importorskip("opendbc.car.rivian.values")
    assert turns.STEER_MAX_LOOKUP == tuple(values.CarControllerParams.STEER_MAX_LOOKUP)

  def test_turn_line_uses_the_logged_limit_and_sent_torque(self, tmp_path):
    ramp = lambda t: -min(385.0, max(0.0, (t - 8.0) * 490))  # noqa: E731  # right turn, torque ramps up from 8 s
    for n in (0, 1):
      write(tmp_path / f"{ROUTE}--{n}" / "rlog.zst", segment(n, settings(), cap=lambda t: 385, can=ramp))
    (d,) = fmlog.load_drives([str(tmp_path)])
    out = run(turns.main, d)
    line = next(x for x in out.splitlines() if "torque:" in x)
    assert "peak sent 385 of 385 limit" in line and "(" not in line  # nothing had to be worked out from the speed curve
    at_limit, climbing = (float(line.split(f"{k} ")[1].split("%")[0]) for k in ("at limit", "climbing"))
    assert at_limit > 70 and 5 < climbing < 25

  def test_old_logs_fall_back_to_the_speed_curve(self, drive_dir):
    (d,) = fmlog.load_drives([str(drive_dir)])
    line = next(x for x in run(turns.main, d).splitlines() if "torque:" in x)
    assert "limit from the speed curve; sent from the logged torque" in line
    assert "peak sent 385 of 385 limit" in line
    assert float(line.split("at limit ")[1].split("%")[0]) > 80  # all of the turn, less the padding around it


class TestTurnTiming:
  """turns.py timing line: when the model, openpilot and the truck reached 30% of the turn's tightest curve."""
  t = np.arange(0, 6, 0.01)

  def ramp(self, start, rise=1.0, peak=0.08):
    return np.clip((self.t - start) / rise, 0, 1) * peak

  def test_onset_time(self):
    x = self.ramp(2.0)
    assert turns.onset_time(self.t, x, 0.3 * 0.08, len(x) - 1) == pytest.approx(2.3, abs=0.011)
    assert np.isnan(turns.onset_time(self.t, x * 0.2, 0.3 * 0.08, len(x) - 1))  # never asked that much
    assert turns.onset_time(self.t, np.full_like(self.t, 1.0), 0.5, len(self.t) - 1) == self.t[0]

  def test_model_late(self):
    truck = self.ramp(2.0)
    q = turns.turn_timing(self.t, self.ramp(3.0), self.ramp(1.5), truck, len(truck) - 1)
    assert q["truck"] == pytest.approx(2.3, abs=0.011)
    assert q["model"] == pytest.approx(1.0, abs=0.02) and q["openpilot"] == pytest.approx(-0.5, abs=0.02)
    assert q["verdict"] == "model late"

  def test_model_never_asked_is_late(self):
    truck = self.ramp(2.0)
    assert turns.turn_timing(self.t, truck * 0.1, truck, truck, len(truck) - 1)["verdict"] == "model late"

  def test_wheel_late(self):
    truck = self.ramp(3.0)
    assert turns.turn_timing(self.t, self.ramp(2.0), self.ramp(2.0), truck, len(truck) - 1)["verdict"] == "wheel late"

  def test_on_time(self):
    truck = self.ramp(2.5)
    assert turns.turn_timing(self.t, self.ramp(2.0), self.ramp(2.0), truck, len(truck) - 1)["verdict"] == "on time"


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


class TestModelRequests:
  @staticmethod
  def drive(lat_active, raw=-0.02, action=0.01, with_raw=True):
    evs = []
    for i in range(40):
      t = i * 0.05
      for kind in ("carControl", "modelV2") + (("modelDataV2SP",) if with_raw else ()):
        e = log.Event.new_message()
        e.logMonoTime = int((1000.0 + t) * 1e9) + (1000 if kind == "modelDataV2SP" else 0)
        m = e.init(kind)
        if kind == "carControl":
          m.latActive = lat_active(t)
        elif kind == "modelV2":
          m.init("action").desiredCurvature = action
        else:
          m.modelCurvature = raw
        evs.append(e)
    return fmlog.Drive("r", [(0, "p", evs)])

  def test_raw_request_while_not_steering(self):
    out = fmlog.model_requests(self.drive(lambda t: t >= 1.0))
    assert out.shape == (40, 2)
    assert np.allclose(out[out[:, 0] < 0.99, 1], -0.02)
    assert np.allclose(out[out[:, 0] > 1.01, 1], 0.01)

  def test_old_logs_keep_the_action(self):
    assert np.allclose(fmlog.model_requests(self.drive(lambda t: False, with_raw=False))[:, 1], 0.01)
    assert np.allclose(fmlog.model_requests(self.drive(lambda t: False, raw=0.0))[:, 1], 0.01)


def _release_case(guard=False):
  """A left turn at 100 Hz: 1 s of you pressing into it (2 Nm) while openpilot sends 400 against you (right), then you
  let go at the wheel's 30 deg; it swings 20 deg past center while 300 decays at 5 per message."""
  n = 300
  t = np.arange(n) * 0.01
  ang = np.concatenate([np.full(100, 300.0), np.linspace(30.0, -20.0, 30), np.linspace(-20.0, 0.0, 170)])
  hand = np.concatenate([np.full(100, 2.0), np.zeros(200)])
  pressed = hand > 1.0
  curv = -np.clip(ang, 0, None) / 3000.0  # right-positive: turning left while the wheel is left
  sent = np.concatenate([np.full(100, -400.0), np.minimum(-300.0 + 5 * np.arange(200), 0.0)])
  lat = np.ones(n, bool)
  ga = np.concatenate([np.ones(100), np.zeros(200)]) if guard else None
  gc = np.concatenate([np.full(100, 2.5), np.zeros(200)]) if guard else None
  return t, ang, hand, pressed, curv, sent, lat, ga, gc


def test_release_check_measures_the_push_against_you_and_past_center():
  t, ang, hand, pressed, curv, sent, lat, _, _ = _release_case()
  r = turns.release_check(t, ang, hand, pressed, curv, sent, lat, 1)
  assert r["press_s"] == pytest.approx(1.0, abs=0.02)
  assert r["against_max"] == 400 and r["against_avg"] == pytest.approx(400)
  assert r["released"] and r["t"] == pytest.approx(1.0)
  assert r["at_release"] == 300
  assert r["crossed"] and r["past_deg"] == pytest.approx(20.0, abs=0.5)
  assert r["push_past_s"] == pytest.approx((60 - 18) * 0.01, abs=0.03)  # crosses center at frame 118, sent 0 at 160
  assert "guard_share" not in r


def test_release_check_reports_the_guard_when_logged():
  t, ang, hand, pressed, curv, sent, lat, ga, gc = _release_case(guard=True)
  r = turns.release_check(t, ang, hand, pressed, curv, sent, lat, 1, ga, gc, guard_logged=True)
  assert r["guard_share"] == pytest.approx(1.0) and r["guard_cut"] == pytest.approx(2.5)


def test_release_check_ignores_short_presses_and_reaction():
  t, ang, hand, pressed, curv, sent, lat, _, _ = _release_case()
  short = pressed.copy()
  short[20:] = False
  assert turns.release_check(t, ang, hand, short, curv, sent, lat, 1) is None
  # hands read against the turn (the wheel's reaction to openpilot's torque, or steering out of it): not a press into it
  assert turns.release_check(t, ang, -hand, -hand > 1.0, curv, sent, lat, 1) is None


def test_release_check_picks_the_long_press_not_a_later_catch():
  t, ang, hand, pressed, curv, sent, lat, _, _ = _release_case()
  hand, pressed, curv = hand.copy(), pressed.copy(), curv.copy()
  hand[250:260] = 2.0  # a short catch later, while the truck turns left a little again
  pressed[250:260] = True
  curv[250:260] = -0.01
  r = turns.release_check(t, ang, hand, pressed, curv, sent, lat, 1)
  assert r["t"] == pytest.approx(1.0)


def assist_events(n, fill):
  """carControlSP events of segment n at 100 Hz; fill(t, turnAssist) sets the fields."""
  evs = []
  for i in range(int(SEG_S * 100)):
    t = n * SEG_S + i / 100 + 0.01
    e = log.Event.new_message()
    e.logMonoTime = int((1000.0 + t) * 1e9)
    fill(t, e.init("carControlSP").turnAssist)
    evs.append(e)
  return evs


class TestAssistLine:
  """turns.py assist line, from carControlSP.turnAssist (right turn 8-13 s, see curvature_at)."""

  @staticmethod
  def fill(t, ta):
    ta.requestIn = curvature_at(t) * 0.5
    ta.requestOut = curvature_at(t)
    if 6.0 <= t < 14.0:
      ta.sign = 1
      ta.source = "carry" if t >= 12.0 else "desire"
      ta.lead = 0.02 if 7.5 <= t < 9.0 else 0.0
      ta.holdFloor = 0.025 if 9.0 <= t < 13.0 else 0.0
      ta.holdActive = t < 11.0
      if t >= 11.0:
        ta.releaseReason = "maxTurned"
        ta.turned = 75.0

  def drive(self, tmp_path, fill):
    prm = settings()
    for n in (0, 1):
      evs = segment(n, prm, loaded="tsfdom_driving_tinygrad.pkl" if n == 0 else None) + assist_events(n, fill)
      write(tmp_path / f"{ROUTE}--{n}" / "rlog.zst", evs)
    (d,) = fmlog.load_drives([str(tmp_path)])
    return d

  def test_assist_line(self, tmp_path):
    out = run(turns.main, self.drive(tmp_path, self.fill))
    line = next(x for x in out.splitlines() if "assist:" in x)
    assert "from blinker + Carry-Through" in line or "from Carry-Through + blinker" in line
    assert "Turn Lead --0 + 7.5s..+1.5 s max 0.020" in line
    assert "Turn Hold floor --0 + 9.0s..+4.0 s max 0.025" in line
    assert "hold released --1 + 1.0s (75 deg limit, 75 deg turned)" in line
    assert "carry --1 + 2.0s..+2.0 s" in line
    assert "kept past the model" in line

  def test_no_turn_desire(self, tmp_path):
    def fill(t, ta):
      ta.requestIn = ta.requestOut = curvature_at(t)
    out = run(turns.main, self.drive(tmp_path, fill))
    assert "assist: none (no turn desire)" in out

  def test_old_logs_have_no_assist_line(self, drive_dir):
    (d,) = fmlog.load_drives([str(drive_dir)])
    assert "assist:" not in run(turns.main, d)


def test_events_are_read_in_time_order(tmp_path):
  evs = segment(0, settings())
  late = evs.pop(5)  # stored out of order, as rlogs sometimes are
  evs.insert(40, late)
  got = [e.logMonoTime for e in fmlog.read(write(tmp_path / "x" / "rlog.zst", evs))]
  assert got == sorted(got)
