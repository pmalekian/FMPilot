"""FM Pilot: FmLateral (fm_lateral.py) runs Lane Guard only when it is the Lane Keeping choice, on top of the request."""
from types import SimpleNamespace

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.lane_guard import TUNES
from openpilot.selfdrive.controls.tests.test_lane_guard import V_EGO, make_model
from openpilot.sunnypilot.selfdrive.controls.lib.fm_lane_keep_mode import MODE_BLENDFUL, MODE_CENTERING, MODE_GUARD, MODE_OFF
from openpilot.sunnypilot.selfdrive.controls.lib.fm_lateral import FmLateral

CP = SimpleNamespace(brand="rivian")
CLOSE_LEFT = make_model(-(1.3 - TUNES["rivian"].half_width + 0.05), lane_width=2.6)  # R1S body edge 5 cm over the left line


def cs(pressed=False):
  return SimpleNamespace(vEgo=V_EGO, steeringPressed=pressed, leftBlinker=False, rightBlinker=False)


def drive(mode, steps=500, lat_active=True, pressed=False, centering=False, desired=0.001):
  params = Params()
  params.put("FM_LaneKeepMode", mode, block=True)
  lat = FmLateral(CP, params)
  for _ in range(steps):
    out = lat.update(desired, CLOSE_LEFT, cs(pressed), lat_active, centering)
  return out - desired, lat


class TestFmLateral(OpenpilotTestCase):
  def test_guard_pushes_away_only_in_guard_mode(self):
    assert drive(MODE_GUARD)[0] > 0.0
    for mode in (MODE_OFF, MODE_CENTERING, MODE_BLENDFUL):
      assert drive(mode)[0] == 0.0, mode

  def test_other_modes_skip_the_update(self):
    _, lat = drive(MODE_OFF, steps=5)
    assert lat.lane_guard_resting

  def test_lane_centering_wins(self):
    assert drive(MODE_GUARD, centering=True)[0] == 0.0

  def test_not_active_passes_through(self):
    assert drive(MODE_GUARD, lat_active=False)[0] == 0.0

  def test_steering_pressed_fades_out(self):
    assert drive(MODE_GUARD, pressed=True)[0] == 0.0
