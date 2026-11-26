import json

import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.fm_jetlink.status import ACTIVE_FRESH, HEARTBEAT_TIMEOUT, ChangeLog, Names, Reader, fill_message

LS2 = "909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8"
FTM = "bc125a9af33033ded89ac1886e27dbbe296da285275b2eb8fc0b5edb35b56c41"
NOW = 5000.0

INDEX = {"models": [{"name": "Light Speed 2 (November 04, 2026)", "ref": "b" * 40, "oid": LS2},
                    {"name": "Fairy tale model (November 18, 2026)", "ref": "5" * 40, "oid": FTM}]}


class Params:
  def __init__(self, **values):
    self.values = values

  def get(self, key):
    value = self.values.get(key)
    if isinstance(value, Exception):
      raise value
    return value


def write(root, name, record):
  (root / name).write_text(record if isinstance(record, str) else json.dumps(record))


def modeld(**kw):
  return {"pid": 1, "at": NOW - 0.4, "stopping": False, "state": "running", "handovers": 1, "active": True,
          "last_output_at": NOW - 0.45, "held": False, "sha256": FTM, "name": "Fairy tale model (November 18, 2026)",
          "checkpoint": "ckpt", "drops": 2, "lags": 3, **kw}


@pytest.fixture
def root(tmp_path):
  (tmp_path / "jetlink").mkdir()
  (tmp_path / "jetlink" / "index.json").write_text(json.dumps(INDEX))
  return tmp_path / "jetlink"


def reader(root, params=None, alive=lambda pid: True):
  return Reader(params or Params(), root=root, index_path=root / "index.json", mono=lambda: NOW, alive=alive)


class TestReader:
  def test_nothing_on_the_device_reads_as_defaults(self, tmp_path):
    d = Reader(Params(), root=tmp_path / "missing", mono=lambda: NOW).read()
    assert d["recordAge"] == -1.0 and d["bigDriving"] is False and d["drivingName"] == "" and d["slotName"] == ""

  def test_big_model_driving_with_everything_filled(self, root):
    write(root, "modeld.json", modeld())
    write(root, "server.json", {"device": "ane-Apple_M2_Pro", "backend": "ort",
                                "pick": {"ref": "5" * 40, "sha256": FTM, "name": None, "at": 1.0}})
    write(root, "status.json", {"link": "usb", "speed": "super-speed", "present": True})
    params = Params(JetlinkSpec={"sha256": LS2, "ready": True},
                    ModelManager_ActiveBundleChestnut={"displayName": "Lebowski Model (July 01, 2026)"},
                    JetlinkProgress={"stage": "ready", "msg": "engine ready"})
    d = reader(root, params).read()
    assert d["bigDriving"] and d["state"] == "running" and d["recordAge"] == 0.4
    assert (d["drivingName"], d["drivingSha256"], d["drivingCheckpoint"]) == ("Fairy tale model (November 18, 2026)", FTM, "ckpt")
    # the Mac's pick is named from the index when the hello carries no name
    assert (d["serverPickName"], d["serverPickSha256"]) == ("Fairy tale model (November 18, 2026)", FTM)
    # what the comma's slot and its record of the last engine built say are separate facts from what drives
    assert d["slotName"] == "Lebowski Model (July 01, 2026)"
    assert (d["builtSha256"], d["builtReady"], d["builtName"]) == (LS2, True, "Light Speed 2 (November 04, 2026)")
    assert (d["device"], d["backend"], d["linkKind"], d["linkSpeed"], d["hostPresent"]) == ("ane-Apple_M2_Pro", "ort", "usb", "super-speed", True)
    assert (d["handovers"], d["drops"], d["lags"], d["held"]) == (1, 2, 3, False)
    assert (d["progressStage"], d["progressMsg"]) == ("ready", "engine ready")

  def test_params_may_arrive_as_json_text_or_bytes(self, root):
    params = Params(JetlinkSpec=json.dumps({"sha256": FTM, "ready": True}),
                    ModelManager_ActiveBundleChestnut=json.dumps({"internalName": "LM"}).encode())
    d = reader(root, params).read()
    assert (d["builtSha256"], d["builtReady"], d["slotName"]) == (FTM, True, "LM")

  def test_the_big_model_is_not_driving_when_its_record_is_stale(self, root):
    write(root, "modeld.json", modeld(at=NOW - ACTIVE_FRESH - 0.5, last_output_at=NOW - ACTIVE_FRESH - 0.55))
    assert reader(root).read()["bigDriving"] is False

  def test_the_big_model_is_not_driving_when_its_last_answer_is_old(self, root):
    write(root, "modeld.json", modeld(last_output_at=NOW - 2.0))
    d = reader(root).read()
    assert d["bigDriving"] is False and d["drivingName"] == ""

  def test_a_stopping_or_dead_modeld_is_not_driving(self, root):
    write(root, "modeld.json", modeld(stopping=True))
    assert reader(root).read()["bigDriving"] is False and reader(root).read()["state"] == ""
    write(root, "modeld.json", modeld())
    assert reader(root, alive=lambda pid: False).read()["bigDriving"] is False

  def test_the_small_model_driving_while_a_pick_is_built(self, root):
    write(root, "modeld.json", modeld(active=False, state="ready", sha256=None, name=None))
    d = reader(root, Params(JetlinkSpec={"sha256": LS2, "ready": False})).read()
    assert d["bigDriving"] is False and d["state"] == "ready" and d["drivingName"] == ""
    assert d["builtReady"] is False and d["builtName"].startswith("Light Speed 2")

  def test_a_name_missing_from_the_record_comes_from_the_index(self, root):
    write(root, "modeld.json", modeld(name=None))
    assert reader(root).read()["drivingName"] == "Fairy tale model (November 18, 2026)"

  def test_a_record_past_the_heartbeat_is_no_modeld(self, root):
    # Jetlink's own status.runtime: a modeld that froze or stopped writing keeps no state
    write(root, "modeld.json", modeld(at=NOW - HEARTBEAT_TIMEOUT - 0.1))
    d = reader(root).read()
    assert d["state"] == "" and d["handovers"] == 0 and d["recordAge"] == round(HEARTBEAT_TIMEOUT + 0.1, 2)

  def test_jetlinks_hash_stand_in_is_not_taken_for_a_name(self, root):
    # a model the comma's index copy doesn't list: Jetlink writes the hash's first 16 characters as its name
    (root / "index.json").write_text(json.dumps({"models": []}))
    write(root, "modeld.json", modeld(name=FTM[:16]))
    assert reader(root).read()["drivingName"] == ""
    # the Mac's pick of the same weights names it
    write(root, "server.json", {"pick": {"sha256": FTM, "name": "Fairy tale model (November 18, 2026)"}})
    d = reader(root, Params(JetlinkSpec={"sha256": FTM, "ready": True})).read()
    assert d["drivingName"] == d["builtName"] == d["serverPickName"] == "Fairy tale model (November 18, 2026)"
    # but never a different model's name
    write(root, "server.json", {"pick": {"sha256": LS2, "name": "Light Speed 2"}})
    assert reader(root).read()["drivingName"] == ""

  def test_junk_records_read_as_defaults(self, root):
    write(root, "modeld.json", "{not json")
    write(root, "server.json", "[1, 2]")
    write(root, "status.json", json.dumps({"link": 5, "speed": None, "present": "yes"}))
    d = reader(root, Params(JetlinkSpec="nonsense", JetlinkProgress=Exception("boom"))).read()
    assert d["bigDriving"] is False and d["device"] == "" and d["linkKind"] == "" and d["builtSha256"] == "" and d["progressStage"] == ""
    write(root, "modeld.json", modeld(handovers=-4, drops=10 ** 9, lags=True, name="x" * 1000))
    d = reader(root).read()
    assert (d["handovers"], d["drops"], d["lags"]) == (0, 65535, 0) and len(d["drivingName"]) == 160

  def test_models_with_the_same_weights_share_a_hash_and_are_both_named(self, root):
    (root / "index.json").write_text(json.dumps({"models": INDEX["models"] + [{"name": "Light Speed 2b", "ref": "c" * 40, "oid": LS2}]}))
    assert Names(root / "index.json").of(LS2) == "Light Speed 2 (November 04, 2026) / Light Speed 2b"

  def test_the_index_is_read_again_when_it_changes(self, root):
    names = Names(root / "index.json")
    assert names.of(FTM).startswith("Fairy")
    (root / "index.json").write_text(json.dumps({"models": [{"name": "Renamed model", "ref": "5" * 40, "oid": FTM}]}))
    assert names.of(FTM) == "Renamed model"
    assert Names(root / "no-such-file.json").of(FTM) == "" and names.of("") == ""


def row(**kw):
  d = {"bigDriving": False, "drivingSha256": "", "drivingName": "", "state": "ready", "device": "ane-Apple_M2_Pro",
       "serverPickSha256": "", "serverPickName": "", "slotName": "Lebowski", "builtSha256": LS2, "builtName": "LS2", "builtReady": True}
  return d | kw


class TestChangeLog:
  def test_a_line_for_the_start_and_one_for_each_change_not_one_per_second(self):
    c = ChangeLog()
    first = c.lines(row())
    assert len(first) == 2 and "small model driving" in first[0] and "last engine built LS2 (909a1400)" in first[1]
    assert c.lines(row(state="joining")) == [] and c.lines(row(state="retrying")) == []  # state flapping stays in the message
    swap = c.lines(row(bigDriving=True, drivingSha256=FTM, drivingName="Fairy tale model", state="running"))
    assert swap == ["FM Pilot Jetlink: big model driving: Fairy tale model (bc125a9a), ane-Apple_M2_Pro"]
    assert c.lines(row(bigDriving=True, drivingSha256=FTM, drivingName="Fairy tale model")) == []

  def test_the_small_model_taking_over_again_says_what_it_was_driving(self):
    c = ChangeLog()
    c.lines(row(bigDriving=True, drivingSha256=FTM, drivingName="Fairy tale model"))
    assert c.lines(row(state="retrying")) == ["FM Pilot Jetlink: small model driving again (Jetlink state retrying), was bc125a9a"]

  def test_a_new_pick_slot_or_engine_is_its_own_line(self):
    c = ChangeLog()
    c.lines(row())
    assert c.lines(row(serverPickSha256=FTM, serverPickName="Fairy tale model")) == [
      "FM Pilot Jetlink: Mac pick Fairy tale model, comma slot Lebowski, last engine built LS2 (909a1400)"]
    assert "not ready" in c.lines(row(serverPickSha256=FTM, serverPickName="Fairy tale model", builtReady=False))[0]

  def test_swapping_between_two_big_models_is_a_change(self):
    c = ChangeLog()
    c.lines(row(bigDriving=True, drivingSha256=LS2, drivingName="LS2"))
    assert "bc125a9a" in c.lines(row(bigDriving=True, drivingSha256=FTM, drivingName="FTM"))[0]


class TestMessage:
  def test_every_field_the_reader_gives_fits_the_message_and_reads_back(self, root):
    write(root, "modeld.json", modeld())
    write(root, "server.json", {"device": "ane-Apple_M2_Pro", "backend": "ort", "pick": {"sha256": FTM, "name": "FTM"}})
    write(root, "status.json", {"link": "usb", "speed": "super-speed", "present": True})
    d = reader(root, Params(JetlinkSpec={"sha256": LS2, "ready": True},
                            ModelManager_ActiveBundleChestnut={"displayName": "Lebowski"},
                            JetlinkProgress={"stage": "ready", "msg": "engine ready"})).read()
    event = log.Event.new_message()
    fill_message(event.init("fmJetlinkStatus"), d)
    with log.Event.from_bytes(event.to_bytes()) as e:
      m = e.fmJetlinkStatus
      for k, v in d.items():
        assert getattr(m, k) == pytest.approx(v), k
    assert set(d) == set(log.Event.schema.fields["fmJetlinkStatus"].schema.fieldnames)

  def test_the_defaults_fit_too(self, tmp_path):
    event = log.Event.new_message()
    fill_message(event.init("fmJetlinkStatus"), Reader(Params(), root=tmp_path / "none", mono=lambda: NOW).read())
    assert event.fmJetlinkStatus.recordAge == -1.0
