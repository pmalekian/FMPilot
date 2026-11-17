import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.sunnypilot.fm_nav import navassistd
from openpilot.sunnypilot.fm_nav.core import State
from openpilot.sunnypilot.fm_nav.roads import Picks, Turn
from openpilot.system.manager.process_config import fm_nav_assist, managed_processes


def roundtrip(state, ms=1.5):
  msg = messaging.new_message("fmNavTurn")
  navassistd.fill_message(msg.fmNavTurn, state, ms)
  with log.Event.from_bytes(msg.to_bytes()) as event:
    return event.fmNavTurn.as_builder().as_reader()


class TestMessage:
  def test_default_is_no_fix_and_no_picks(self):
    m = roundtrip(State())
    assert str(m.status) == "noFix"
    assert not (m.left.valid or m.right.valid or m.leftService.valid or m.rightService.valid)

  def test_picks(self):
    left = Picks(road="Main Street", road_name="Main Street", road_kind="residential", cross_track=1.5,
                 services=[Turn(20.0, -88.0, "", "service", True)])
    right = Picks(road="Main Street", road_name="Main Street", road_kind="residential",
                  streets=[Turn(42.0, 91.0, "Oak Lane", "residential", False), Turn(70.0, 85.0, "Elm Road", "tertiary", False)])
    m = roundtrip(State(status="ok", heading=12.0, gps_accuracy=2.0, left=left, right=right, tiles=2))
    assert (str(m.status), m.roadName, m.roadKind, m.onService, m.tiles) == ("ok", "Main Street", "residential", False, 2)
    assert (m.right.valid, m.right.name, m.right.count) == (True, "Oak Lane", 2)
    assert (round(m.right.distance), round(m.right.angle), round(m.right.nextGap)) == (42, 91, 28)
    assert not m.left.valid
    assert m.leftService.valid and round(m.leftService.distance) == 20 and m.leftService.kind == "service"
    assert abs(m.computeMs - 1.5) < 1e-6

  def test_every_status_fits_the_message(self):
    from openpilot.sunnypilot.fm_nav.core import STATUS
    for status in STATUS:
      assert str(roundtrip(State(status=status)).status) == status


class TestProcess:
  def test_no_map_folder(self, tmp_path):
    assert navassistd.open_map(str(tmp_path)) is None
    assert navassistd.open_map(str(tmp_path / "missing")) is None

  def test_unreadable_map_is_skipped(self, tmp_path):
    (tmp_path / "bad.pmtiles").write_bytes(b"x" * 500)
    assert navassistd.open_map(str(tmp_path)) is None

  def test_runs_only_onroad_with_the_setting_on(self):
    class P:
      def __init__(self, on):
        self.on = on

      def get_bool(self, key):
        assert key == "FM_NavAssist"
        return self.on
    assert fm_nav_assist(True, P(True), None)
    assert not fm_nav_assist(True, P(False), None)
    assert not fm_nav_assist(False, P(True), None)
    assert "fm_navassistd" in managed_processes
