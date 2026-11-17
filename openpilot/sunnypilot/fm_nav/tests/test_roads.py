import gzip
import math
import struct

import numpy as np
import pytest

from openpilot.sunnypilot.fm_nav.pmtiles import PMTiles, LocalFile, tile_id
from openpilot.sunnypilot.fm_nav.roads import RANGE, RoadMap
from openpilot.sunnypilot.fm_nav.vector_tile import decode_roads

MAIN = ([(0, -300), (0, 300)], "Main Street", "residential")  # runs north
P0 = np.array([0.0, 0.0])


def street(y, x0=0, x1=120, name="Oak Lane", kind="residential"):
  return ([(x0, y), (x1, y)], name, kind)


def names(turns):
  return [t.name or t.kind for t in turns]


class TestPick:
  def test_side_street_on_the_right(self):
    m = RoadMap.from_roads([MAIN, street(50)])
    right, left = m.pick(P0, 0.0, 1), m.pick(P0, 0.0, -1)
    assert right.road_name == "Main Street" and not right.road_service
    assert names(right.streets) == ["Oak Lane"]
    assert right.streets[0].distance == pytest.approx(50, abs=2.5)
    assert right.streets[0].angle == pytest.approx(90, abs=3)
    assert left.streets == [] and left.services == []

  def test_same_street_is_on_the_left_driving_the_other_way(self):
    m = RoadMap.from_roads([MAIN, street(50)])
    p = np.array([0.0, 100.0])
    assert names(m.pick(p, 180.0, -1).streets) == ["Oak Lane"]
    assert m.pick(p, 180.0, 1).streets == []

  def test_out_of_range_and_behind(self):
    m = RoadMap.from_roads([MAIN, street(50)])
    assert m.pick(np.array([0.0, 50 - RANGE - 5]), 0.0, 1).streets == []
    assert m.pick(np.array([0.0, 60.0]), 0.0, 1).streets == []  # passed it

  def test_crossing_gives_both_sides(self):
    m = RoadMap.from_roads([MAIN, street(40, x0=-120, x1=120, name="Elm Road")])
    assert names(m.pick(P0, 0.0, 1).streets) == ["Elm Road"]
    assert names(m.pick(P0, 0.0, -1).streets) == ["Elm Road"]
    assert m.pick(P0, 0.0, -1).streets[0].angle == pytest.approx(-90, abs=3)

  def test_service_roads_are_reported_apart(self):
    m = RoadMap.from_roads([MAIN, street(30, name="", kind="service"), street(60)])
    picks = m.pick(P0, 0.0, 1)
    assert names(picks.streets) == ["Oak Lane"]
    assert names(picks.services) == ["service"]
    assert picks.services[0].distance == pytest.approx(30, abs=2.5) and picks.services[0].service

  def test_two_streets_nearest_first(self):
    m = RoadMap.from_roads([MAIN, street(60, name="Second Avenue"), street(25, name="First Avenue")])
    turns = m.pick(P0, 0.0, 1).streets
    assert names(turns) == ["First Avenue", "Second Avenue"]
    assert turns[1].distance - turns[0].distance == pytest.approx(35, abs=3)

  def test_a_gentle_fork_is_not_a_turn(self):
    fork = ([(0, 40), (40, 150)], "Fork Road", "residential")  # 20 deg off the truck's heading
    assert RoadMap.from_roads([MAIN, fork]).pick(P0, 0.0, 1).streets == []

  def test_a_parallel_road_is_not_a_junction(self):
    other_half = ([(20, -300), (20, 300)], "Main Street East", "residential")
    m = RoadMap.from_roads([MAIN, other_half])
    assert m.pick(P0, 0.0, 1).streets == [] and m.pick(P0, 0.0, -1).streets == []

  def test_a_street_that_is_not_on_my_road(self):
    # it meets a parallel road 60 m to the right, not the road the truck is on
    m = RoadMap.from_roads([MAIN, ([(60, -300), (60, 300)], "Far Street", "residential"), street(50, x0=60, x1=200)])
    assert m.pick(P0, 0.0, 1).streets == []

  def test_a_street_made_of_two_pieces_is_one_junction(self):
    m = RoadMap.from_roads([MAIN, street(50, x0=-120, x1=0, name="Oak Lane"), street(50, x0=0, x1=120, name="Oak Lane")])
    assert len(m.pick(P0, 0.0, 1).streets) == 1

  def test_off_the_road(self):
    m = RoadMap.from_roads([MAIN, street(50)])
    assert m.pick(np.array([40.0, 0.0]), 0.0, 1).road == ""  # 40 m from any road
    assert m.pick(P0, 90.0, 1).road == ""  # on Main Street but pointing across it
    assert m.current_road(P0, 180.0)[0] == 0  # either way along it counts


# The archive and tile builders are from commaai/openpilot master, selfdrive/navigation/tests/test_navigation.py (MIT).
def varint(value):
  output = bytearray()
  while value > 127:
    output.append((value & 127) | 128)
    value >>= 7
  output.append(value)
  return bytes(output)


def field(number, value):
  if isinstance(value, bytes):
    return varint(number << 3 | 2) + varint(len(value)) + value
  return varint(number << 3) + varint(value)


def road_tile():
  # one named vertical line from (2048, 0) to (2048, 4096), tagged oneway, and one non-road layer
  geometry = b''.join(varint(v) for v in [9, 4096, 0, 10, 0, 8192])
  feature = field(2, b'\x00\x00\x01\x01\x02\x02') + field(3, 2) + field(4, geometry)
  roads = field(15, 2) + field(1, b'roads') + field(2, feature)
  roads += field(3, b'kind') + field(3, b'name') + field(3, b'oneway')
  roads += field(4, field(1, b'minor_road')) + field(4, field(1, b'Oak Lane')) + field(4, field(7, 1)) + field(5, 4096)
  return field(3, field(1, b'buildings')) + field(3, roads)


def make_archive(z=0, x=0, y=0):
  tile = gzip.compress(road_tile())
  root = gzip.compress(varint(1) + varint(tile_id(z, x, y)) + varint(1) + varint(len(tile)) + varint(1))
  metadata = gzip.compress(b'{}')
  header = bytearray(127)
  header[:8] = b'PMTiles\x03'
  offsets = [127, 127 + len(root), 127 + len(root) + len(metadata), 127 + len(root) + len(metadata)]
  struct.pack_into('<11Q', header, 8, offsets[0], len(root), offsets[1], len(metadata), offsets[2], 0, offsets[3], len(tile), 1, 1, 1)
  header[96:102] = bytes([1, 2, 2, 1, 0, z])
  return bytes(header + root + metadata + tile)


class TestArchive:
  def test_road_names_and_oneway(self):
    roads = decode_roads(road_tile())
    assert [(r.kind, r.name, r.oneway) for r in roads] == [("minor_road", "Oak Lane", True)]
    np.testing.assert_array_equal(roads[0].points, [[0.5, 0], [0.5, 1]])

  def test_archive_on_disk(self, tmp_path):
    path = tmp_path / "map.pmtiles"
    path.write_bytes(make_archive())
    archive = PMTiles(LocalFile(str(path)))
    assert archive.get(0, 0, 0) == road_tile()
    archive.close()

  def test_road_map_loads_tiles_around_the_truck(self, tmp_path):
    path = tmp_path / "map.pmtiles"
    path.write_bytes(make_archive())
    m = RoadMap(str(path))
    assert m.zoom == 0
    assert m.update(0.0, 10.0) and not m.update(0.0, 10.0)  # loaded once
    assert m.names == ["Oak Lane"] and len(m.xy) > 100
    assert m.current_road(m.local(0.0, 10.0)[0], 0.0) is not None  # the line runs along longitude 0
    m.close()

  def test_tile_lands_where_it_is_on_the_earth(self, tmp_path):
    # the zoom 15 tile over Bartlett, IL (41.95 N, 88.178 W); its one road runs north-south through its middle
    z = 15
    tx = int((-88.178 + 180) / 360 * 2 ** z)
    ty = int((1 - math.asinh(math.tan(math.radians(41.95))) / math.pi) / 2 * 2 ** z)
    path = tmp_path / "map.pmtiles"
    path.write_bytes(make_archive(z, tx, ty))
    m = RoadMap(str(path))
    lon = (tx + 0.5) / 2 ** z * 360 - 180
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * (ty + 0.5) / 2 ** z))))
    assert abs(lat - 41.95) < 0.01 and abs(lon + 88.178) < 0.01
    assert m.update(lon, lat)
    here = m.local(lon, lat)[0]
    assert m.current_road(here, 0.0)[1] < 1.0  # on the line
    assert m.current_road(here + [10.0, 0.0], 0.0)[1] == pytest.approx(10.0, abs=1.0)  # 10 m east of it
    assert np.ptp(m.xy[:, 0]) < 1.0 and 850 < np.ptp(m.xy[:, 1]) < 950  # a z15 tile is about 910 m tall here
    assert m.current_road(here + [0.0, 300.0], 0.0) is not None and m.current_road(here + [0.0, 600.0], 0.0) is None
    m.close()

  def test_not_an_archive(self, tmp_path):
    path = tmp_path / "bad.pmtiles"
    path.write_bytes(b"not a map" * 100)
    with pytest.raises(ValueError):
      RoadMap(str(path))
