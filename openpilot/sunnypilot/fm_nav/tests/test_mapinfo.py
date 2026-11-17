"""The map check behind FM_NavMapInfo: a good map, no map, a damaged one, one that doesn't cover the truck."""
import os

from openpilot.sunnypilot.fm_nav import mapinfo, navmapd
from openpilot.sunnypilot.fm_nav import extract as X
from openpilot.sunnypilot.fm_nav.pmtiles import PMTiles, LocalFile
from openpilot.sunnypilot.fm_nav.tests.test_extract import LAT, LON, FakeParams, build_source


def make(tmp_path, radius_m=3000, meta=None, name="fm-nav.pmtiles"):
  src = tmp_path / "src.pmtiles"
  build_source(src, radius_m=radius_m)
  out = tmp_path / "fmnav"
  os.makedirs(out, exist_ok=True)
  summary = X.extract(PMTiles(LocalFile(str(src))), LAT, LON, radius_m, str(out / name), metadata=meta or {})
  return str(out), summary


def test_good_map(tmp_path):
  d, _ = make(tmp_path, meta={"fm_build": "20261115"})
  ok, text = mapinfo.describe(d, (LAT, LON))
  assert ok and text.startswith("Map OK: fm-nav.pmtiles, ")
  assert "map of 2026-11-15" in text and "2 mi around 41.995, -88.186" in text and "road pieces around" in text
  assert "outside" not in text


def test_no_position_yet(tmp_path):
  d, _ = make(tmp_path)
  ok, text = mapinfo.describe(d, None)
  assert ok and "No GPS position saved yet" in text


def test_no_map(tmp_path):
  ok, text = mapinfo.describe(str(tmp_path), (LAT, LON))
  assert not ok and text.startswith("No map.")


def test_unreadable_map(tmp_path):
  (tmp_path / "fm-nav.pmtiles").write_bytes(b"not a map" * 50)
  ok, text = mapinfo.describe(str(tmp_path), (LAT, LON))
  assert not ok and text.startswith("Can't read fm-nav.pmtiles")


def test_truck_outside_the_map(tmp_path):
  d, _ = make(tmp_path)
  ok, text = mapinfo.describe(d, (LAT + 1.0, LON))  # about 69 mi north: no tile there
  assert not ok and text.startswith("Map has no roads where the truck last was") and "outside it" in text


def test_copied_file_without_our_metadata(tmp_path):
  d, _ = make(tmp_path)
  ok, text = mapinfo.describe(d, (LAT, LON))
  assert ok and "fm-nav.pmtiles" in text  # a build date is only known for Download Nav Map's files


def test_publish_writes_once_and_signature_follows_the_file(tmp_path):
  d, _ = make(tmp_path)
  p = FakeParams(LastGPSPositionLLK=f'{{"latitude": {LAT}, "longitude": {LON}}}')
  puts = []
  real_put = p.put
  p.put = lambda k, v: (puts.append(k), real_put(k, v))
  sig = mapinfo.signature(p, d)
  text = mapinfo.publish(p, d)
  mapinfo.publish(p, d)
  assert p.kv["FM_NavMapInfo"] == text and puts == ["FM_NavMapInfo"]
  assert mapinfo.signature(p, d) == sig
  os.utime(os.path.join(d, "fm-nav.pmtiles"), (1, 1))
  assert mapinfo.signature(p, d) != sig


def test_download_leaves_a_checked_map(tmp_path):
  """What Download Nav Map writes is what the check reads: build date and area come from the archive itself."""
  d, _ = make(tmp_path, meta={"fm_build": "20261114"})
  assert "map of 2026-11-14" in mapinfo.describe(d, (LAT, LON))[1]
  assert navmapd.last_position(FakeParams(LastGPSPositionLLK='{"latitude": 42.0, "longitude": -88.0}')) == (42.0, -88.0)
