"""Extract and Download Nav Map, against a small archive built here and served over HTTP with range requests."""
import datetime
import gzip
import json
import math
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from openpilot.sunnypilot.fm_nav import extract as X
from openpilot.sunnypilot.fm_nav import navmapd
from openpilot.sunnypilot.fm_nav.pmtiles import PMTiles, LocalFile, decode_directory, tile_id
from openpilot.sunnypilot.fm_nav.roads import RoadMap
from openpilot.sunnypilot.fm_nav.vector_tile import decode_roads

LAT, LON = 41.995, -88.186  # Bartlett, IL


def field(number, value):
  if isinstance(value, bytes):
    return X.write_varint(number << 3 | 2) + X.write_varint(len(value)) + value
  return X.write_varint(number << 3) + X.write_varint(value)


def layer(name, road_name=None):
  geometry = b''.join(X.write_varint(v) for v in [9, 4096, 0, 10, 0, 8192])  # one north-south line through the middle
  feature = field(2, b'\x00\x00') + field(3, 2) + field(4, geometry)
  out = field(15, 2) + field(1, name) + field(2, feature) + field(3, b'kind') + field(4, field(1, b'minor_road')) + field(5, 4096)
  if road_name:
    feature = field(2, b'\x00\x00\x01\x01') + field(3, 2) + field(4, geometry)
    out = field(15, 2) + field(1, name) + field(2, feature) + field(3, b'kind') + field(3, b'name') + \
      field(4, field(1, b'minor_road')) + field(4, field(1, road_name)) + field(5, 4096)
  return out


def tile_bytes(x, y, roads=True):
  body = field(3, layer(b'buildings')) + field(3, layer(b'earth'))
  if roads:
    body += field(3, layer(b'roads', f"Road {x}-{y}".encode()))
  return gzip.compress(body, mtime=0)


def build_source(path, radius_m=6000, empty_every=7):
  """A z0-15 world archive with tiles at zoom 15 around Bartlett (some without roads) and one at zoom 14."""
  xy = X.tiles_in_circle(LAT, LON, radius_m)
  tiles = {tile_id(15, x, y): tile_bytes(x, y, roads=(i % empty_every != 0)) for i, (x, y) in enumerate(xy)}
  tiles[tile_id(14, xy[0][0] // 2, xy[0][1] // 2)] = tile_bytes(0, 0)
  data, entries = b'', []
  for tid in sorted(tiles):
    entries.append((tid, 1, len(data), len(tiles[tid])))
    data += tiles[tid]
  X.LEAF_ENTRIES, saved = 64, X.LEAF_ENTRIES  # force leaf directories in the source too
  data_path = str(path) + ".data"
  with open(data_path, 'wb') as f:
    f.write(data)
  try:
    X._write_archive(str(path), data_path, {e[0]: (e[2], e[3]) for e in entries}, LAT, LON, radius_m, {})
  finally:
    X.LEAF_ENTRIES = saved
    os.remove(data_path)
  # widen the zoom range to 0-15, like a world build
  with open(path, 'r+b') as f:
    f.seek(100)
    f.write(bytes([0, 15]))
  return xy, tiles


class RangeServer:
  def __init__(self, files):
    self.files, self.requests, self.ignore_range, self.full_bodies = files, [], False, 0
    outer = self

    class H(BaseHTTPRequestHandler):
      def log_message(self, *a):
        pass

      def _file(self):
        name = self.path.lstrip('/')
        return outer.files.get(name)

      def do_HEAD(self):
        p = self._file()
        if p is None:
          self.send_response(404)
          self.end_headers()
          return
        self.send_response(200)
        self.send_header('Content-Length', str(os.path.getsize(p)))
        self.end_headers()

      def do_GET(self):
        p = self._file()
        if outer.ignore_range:  # a server that sends the whole file
          self.send_response(200)
          self.send_header('Content-Length', str(os.path.getsize(p)))
          self.end_headers()
          outer.full_bodies += 1
          with open(p, 'rb') as f:
            self.wfile.write(f.read())
          return
        a, b = (int(v) for v in self.headers['Range'].split('=')[1].split('-'))
        outer.requests.append((a, b))
        with open(p, 'rb') as f:
          f.seek(a)
          body = f.read(b - a + 1)
        self.send_response(206)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    self.httpd = ThreadingHTTPServer(('127.0.0.1', 0), H)
    self.url = f"http://127.0.0.1:{self.httpd.server_port}/"
    threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

  def close(self):
    self.httpd.shutdown()


@pytest.fixture
def source(tmp_path):
  path = tmp_path / "world.pmtiles"
  xy, tiles = build_source(path)
  return path, xy, tiles


class TestExtract:
  def test_varint_and_directory_roundtrip(self):
    entries = [(5, 1, 0, 10), (6, 1, 10, 4), (9, 3, 100, 7), (20, 1, 14, 2)]
    ids, decoded = decode_directory(gzip.decompress(X.encode_directory(entries)))
    assert ids == [5, 6, 9, 20] and decoded == [(1, 10, 0), (1, 4, 10), (3, 7, 100), (1, 2, 14)]

  def test_circle(self):
    xy = X.tiles_in_circle(LAT, LON, 50 * X.MILE)
    xs, ys = [p[0] for p in xy], [p[1] for p in xy]
    # a z15 tile is ~900 m here: about 180 tiles across, and a circle, not its square
    assert 170 < max(xs) - min(xs) < 200 and 0.75 < len(xy) / ((max(xs) - min(xs) + 1) * (max(ys) - min(ys) + 1)) < 0.83

  def test_keep_roads(self):
    cut = X.keep_roads(tile_bytes(1, 2))
    assert [r.name for r in decode_roads(gzip.decompress(cut))] == ["Road 1-2"]
    assert len(cut) < len(tile_bytes(1, 2))
    assert X.keep_roads(tile_bytes(1, 2, roads=False)) is None

  def test_extract_from_a_file(self, source, tmp_path):
    path, xy, tiles = source
    out = tmp_path / "out.pmtiles"
    s = X.extract(PMTiles(LocalFile(str(path))), LAT, LON, 3000, str(out))
    want = X.tiles_in_circle(LAT, LON, 3000)
    a = PMTiles(LocalFile(str(out)))
    assert (a.min_zoom, a.max_zoom) == (15, 15)
    for i, (x, y) in enumerate(xy):
      t = a.get(15, x, y)
      if (x, y) not in want or i % 7 == 0:
        assert t is None  # outside the circle, or a tile without roads
      else:
        assert [r.name for r in decode_roads(t)] == [f"Road {x}-{y}"]
    assert s["tiles"] == len([p for i, p in enumerate(xy) if p in want and i % 7]) and s["wanted"] == len(want)
    a.close()
    m = RoadMap(str(out))
    assert m.update(LON, LAT) and m.names[0].startswith("Road ")  # Nav Assist reads it
    m.close()

  def test_cancel(self, source, tmp_path):
    path, _, _ = source
    with pytest.raises(X.Cancelled):
      X.extract(PMTiles(LocalFile(str(path))), LAT, LON, 3000, str(tmp_path / "o.pmtiles"), cancelled=lambda: True)
    assert not os.path.exists(tmp_path / "o.pmtiles.data")

  def test_over_http_with_few_requests(self, source, tmp_path):
    path, _, _ = source
    srv = RangeServer({"world.pmtiles": str(path)})
    try:
      s = X.extract(PMTiles(X.HttpSource(srv.url + "world.pmtiles")), LAT, LON, 3000, str(tmp_path / "h.pmtiles"))
      assert s["tiles"] > 20 and len(srv.requests) < 15  # header, directories, a few merged tile reads
    finally:
      srv.close()


  def test_a_server_that_ignores_range_is_refused(self, source):
    path, _, _ = source
    srv = RangeServer({"world.pmtiles": str(path)})
    srv.ignore_range = True
    try:
      src = X.HttpSource(srv.url + "world.pmtiles")
      with pytest.raises(ValueError, match="answered 200"):
        src.read(0, 127)
    finally:
      srv.close()


class FakeParams:
  def __init__(self, **kv):
    self.kv = dict(kv)

  def get(self, key, return_default=False):
    return self.kv.get(key)

  def get_bool(self, key):
    return bool(self.kv.get(key))

  def put(self, key, value):
    self.kv[key] = value

  def put_bool(self, key, value):
    self.kv[key] = bool(value)


class TestDownloader:
  def make(self, source, tmp_path, monkeypatch, **params):
    path, _, _ = source
    srv = RangeServer({"20261115.pmtiles": str(path)})  # today's (16th) build not up yet
    monkeypatch.setattr(navmapd, "BUILD_URL", srv.url + "{}.pmtiles")
    monkeypatch.setattr(navmapd, "RADIUS_MI", 2)
    p = FakeParams(FM_NavMapDownload=True, LastGPSPositionLLK=json.dumps({"latitude": LAT, "longitude": LON}), **params)
    p.kv.setdefault("IsOffroad", True)
    d = navmapd.Downloader(p, map_dir=str(tmp_path / "fmnav"), now=lambda: datetime.datetime(2026, 11, 16, 7, 0))
    return d, p, srv

  def test_downloads_and_replaces_the_old_map(self, source, tmp_path, monkeypatch):
    d, p, srv = self.make(source, tmp_path, monkeypatch)
    os.makedirs(tmp_path / "fmnav")
    (tmp_path / "fmnav" / "fm-chicagoland.pmtiles").write_bytes(b"old")
    try:
      d.step()
    finally:
      srv.close()
    assert sorted(os.listdir(tmp_path / "fmnav")) == ["fm-nav.pmtiles"]
    assert p.kv["FM_NavMapStatus"].startswith("Ready: 2 mi around 41.995, -88.186, map of 2026-11-15")
    assert p.kv["FM_NavMapDownload"] is False
    assert X.check_archive(str(tmp_path / "fmnav" / "fm-nav.pmtiles"), LAT, LON) > 0

  def test_no_position(self, source, tmp_path, monkeypatch):
    d, p, srv = self.make(source, tmp_path, monkeypatch)
    del p.kv["LastGPSPositionLLK"]
    try:
      d.step()
    finally:
      srv.close()
    assert p.kv["FM_NavMapStatus"].startswith("Failed: no position") and p.kv["FM_NavMapDownload"] is False

  def test_position_param(self):
    assert navmapd.last_position(FakeParams(LastGPSPositionLLK='{"latitude": 42.0, "longitude": -88.0, "altitude": 200}')) == (42.0, -88.0)
    for bad in ('{"latitude": 0, "longitude": 0}', '{"latitude": 42.0}', 'not json', None):
      assert navmapd.last_position(FakeParams(LastGPSPositionLLK=bad)) is None

  def test_no_build_keeps_the_old_map(self, source, tmp_path, monkeypatch):
    d, p, srv = self.make(source, tmp_path, monkeypatch)
    srv.files.clear()
    os.makedirs(tmp_path / "fmnav")
    (tmp_path / "fmnav" / "fm-nav.pmtiles").write_bytes(b"old")
    try:
      d.step()
    finally:
      srv.close()
    assert (tmp_path / "fmnav" / "fm-nav.pmtiles").read_bytes() == b"old"
    assert p.kv["FM_NavMapStatus"].startswith("Failed: no Protomaps build") and p.kv["FM_NavMapDownload"] is False

  def test_driving_stops_it_and_leaves_the_setting_on(self, source, tmp_path, monkeypatch):
    d, p, srv = self.make(source, tmp_path, monkeypatch, IsOffroad=False)
    try:
      d.step()
    finally:
      srv.close()
    assert p.kv["FM_NavMapDownload"] is True and p.kv["FM_NavMapStatus"].startswith("Stopped: started driving")
    assert not any(n.endswith(".pmtiles") for n in os.listdir(tmp_path / "fmnav"))

  def test_turning_it_off_mid_download(self, source, tmp_path, monkeypatch):
    d, p, srv = self.make(source, tmp_path, monkeypatch)
    real = X.extract

    def off_first(*a, **kw):
      p.kv["FM_NavMapDownload"] = False
      return real(*a, **kw)
    monkeypatch.setattr(X, "extract", off_first)
    try:
      d.step()
    finally:
      srv.close()
    assert p.kv["FM_NavMapStatus"] == "Stopped." and p.kv["FM_NavMapDownload"] is False

  def test_not_enough_space(self, source, tmp_path, monkeypatch):
    d, p, srv = self.make(source, tmp_path, monkeypatch)
    monkeypatch.setattr(navmapd, "SPACE_MARGIN", 10 ** 15)
    try:
      d.step()
    finally:
      srv.close()
    assert "not enough space" in p.kv["FM_NavMapStatus"]
    assert not any(n.endswith(".pmtiles") for n in os.listdir(tmp_path / "fmnav"))

  def test_idle_does_nothing(self, tmp_path):
    p = FakeParams()
    navmapd.Downloader(p, map_dir=str(tmp_path / "x")).step()
    assert p.kv == {} and not os.path.exists(tmp_path / "x")


def test_newest_map_is_used(tmp_path):
  from openpilot.sunnypilot.fm_nav import navassistd
  src = tmp_path / "s.pmtiles"
  build_source(src, radius_m=1500)
  old, new = tmp_path / "a-old.pmtiles", tmp_path / "z-new.pmtiles"
  for f in (old, new):
    f.write_bytes(src.read_bytes())
  os.utime(old, (1, 1))
  m = navassistd.open_map(str(tmp_path))
  assert m.archive.source.f.name == str(new) or math.isclose(os.path.getmtime(m.archive.source.f.name), os.path.getmtime(new))
  m.close()


def test_every_param_the_nav_features_use_exists():
  """A key missing from params_keys.h raises on the device; the fakes in these tests would hide it."""
  import re
  root = os.path.join(os.path.dirname(__file__), "..", "..", "..")
  keys = set(re.findall(r'\{"(\w+)", \{', open(os.path.join(root, "common", "params_keys.h")).read()))
  files = ["sunnypilot/fm_nav/navmapd.py", "sunnypilot/fm_nav/navassistd.py", "system/manager/process_config.py",
           "sunnypilot/selfdrive/controls/lib/fm_nav_turn_desire.py", "sunnypilot/selfdrive/controls/lib/fm_nav_turn_speed.py",
           "selfdrive/ui/sunnypilot/ui_state.py"]
  used = set()
  for f in files:
    text = open(os.path.join(root, f)).read()
    used |= set(re.findall(r'params\.(?:get_bool|get|put|put_bool)\("(\w+)"', text))
    used |= set(re.findall(r'^PARAM_\w+ = "(\w+)"', text, re.M)) | set(re.findall(r'^POSITION_PARAM = "(\w+)"', text, re.M))
  assert {"FM_NavAssist", "FM_NavTurnDesire", "FM_NavTurnSpeed", "FM_NavTurnSpeedMph", "FM_NavMapDownload", "FM_NavMapStatus",
          "FM_TopReadout", "IsOffroad", "LastGPSPositionLLK"} <= used
  assert used - keys == set()  # every key these files read or write, FM's or not
