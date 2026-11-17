import math

import pytest

from openpilot.sunnypilot.fm_nav.core import GPS_MAX_AGE, MAX_SPEED, NavAssist
from openpilot.sunnypilot.fm_nav.roads import RoadMap

ROADS = [([(0, -300), (0, 300)], "Main Street", "residential"), ([(0, 50), (120, 50)], "Oak Lane", "residential")]


def make():
  m = RoadMap.from_roads(ROADS)
  return NavAssist(m), m


def gps(nav, m, t, x, y, speed=8.0, bearing=0.0, accuracy=2.0):
  nav.on_gps(t, x / m.kx, y / m.ky, speed, bearing, accuracy)


class TestNavAssist:
  def test_no_map(self):
    assert NavAssist(None).tick(0.0).status == "noMap"

  def test_needs_a_fix_then_a_heading(self):
    nav, m = make()
    assert nav.tick(0.0).status == "noFix"
    gps(nav, m, 0.0, 0, 0, speed=0.5)  # too slow for the GPS course to mean anything
    assert nav.tick(0.1).status == "noHeading"
    gps(nav, m, 0.2, 0, 0, speed=8.0)
    s = nav.tick(0.3)
    assert s.status == "ok" and s.heading == pytest.approx(0.0, abs=0.1)
    assert [t.name for t in s.right.streets] == ["Oak Lane"] and s.left.streets == []

  def test_fix_goes_stale(self):
    nav, m = make()
    gps(nav, m, 0.0, 0, 0)
    assert nav.tick(GPS_MAX_AGE - 0.1).status == "ok"
    assert nav.tick(GPS_MAX_AGE + 0.1).status == "noFix"

  def test_poor_fix_is_ignored(self):
    nav, m = make()
    gps(nav, m, 0.0, 0, 0, accuracy=25.0)
    assert nav.tick(0.1).status == "noFix"

  def test_too_fast(self):
    nav, m = make()
    gps(nav, m, 0.0, 0, 0)
    nav.on_car(MAX_SPEED + 1)
    assert nav.tick(0.1).status == "tooFast"

  def test_off_road(self):
    nav, m = make()
    gps(nav, m, 0.0, 60, -100)
    assert nav.tick(0.1).status == "offRoad"

  def test_position_is_carried_forward_between_fixes(self):
    nav, m = make()
    gps(nav, m, 0.0, 0, 0)
    nav.on_car(10.0)
    near = nav.tick(0.0).right.streets[0].distance
    assert nav.tick(1.0).right.streets[0].distance == pytest.approx(near - 10.0, abs=0.5)

  def test_heading_follows_the_yaw_rate_at_a_stop(self):
    # GPS course set while moving north; then stopped, the truck turns 90 deg right on the spot: no new course
    nav, m = make()
    gps(nav, m, 0.0, 0, 0, speed=8.0, bearing=0.0)
    nav.on_pose(0.0, 0.0)
    t = 0.0
    for _ in range(40):
      t += 0.05
      nav.on_pose(t, math.radians(45))
      gps(nav, m, t, 0, 0, speed=0.2, bearing=270.0)  # a stopped receiver's course is noise
    assert nav.heading == pytest.approx(90.0, abs=0.5)

  def test_gps_course_pulls_heading_back(self):
    nav, m = make()
    gps(nav, m, 0.0, 0, 0, bearing=0.0)
    nav.yaw_sum += 20.0  # a drifted sum
    for i in range(200):
      gps(nav, m, 0.1 * i, 0, 0, bearing=0.0)
    assert min(nav.heading, 360 - nav.heading) < 0.5

  def test_bad_yaw_rate_is_skipped(self):
    nav, _ = make()
    nav.on_pose(0.0, 0.0)
    nav.on_pose(0.05, float("nan"))
    nav.on_pose(5.0, 1.0)  # a long gap counts as 0.2 s at most
    assert nav.yaw_sum == pytest.approx(math.degrees(1.0) * 0.2)
