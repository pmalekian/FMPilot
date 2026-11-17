"""
FM Pilot Nav Assist: cut a road map for a circle around a point out of a Protomaps basemap.

Reads only what it needs from the source archive (a daily world build on build.protomaps.com, through HTTP range
requests, or a file): its directories, then the zoom 15 tiles that touch the circle. From each tile it keeps only the
"roads" layer (all Nav Assist reads; buildings, land use and places are most of a suburban tile), and writes a new
PMTiles v3 archive of those tiles.

Map data: OpenStreetMap contributors (ODbL) through Protomaps basemaps.
"""
import gzip
import json
import math
import os
import struct
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor

from openpilot.sunnypilot.fm_nav.pmtiles import HEADER_LENGTH, PMTiles, decode_directory, tile_id
from openpilot.sunnypilot.fm_nav.vector_tile import fields

ZOOM = 15
MILE = 1609.344
EARTH_RADIUS = 6371008.8
MERGE_GAP = 256 * 1024  # bytes between two wanted tiles below which one request reads both
MAX_REQUEST = 8 * 1024 * 1024  # bytes per range request
ROOT_MAX = 16384 - HEADER_LENGTH  # PMTiles: header and root directory fit in the first 16 KiB
LEAF_ENTRIES = 4096
KEEP_LAYERS = (b'roads',)


class Cancelled(Exception):
  pass


def write_varint(value: int) -> bytes:
  out = bytearray()
  while value > 127:
    out.append((value & 127) | 128)
    value >>= 7
  out.append(value)
  return bytes(out)


def encode_directory(entries: list[tuple[int, int, int, int]]) -> bytes:
  """entries: (tile id, run length, offset, length), sorted by tile id. Returns the gzip-compressed directory."""
  out = bytearray(write_varint(len(entries)))
  last = 0
  for tid, _, _, _ in entries:
    out += write_varint(tid - last)
    last = tid
  for _, run, _, _ in entries:
    out += write_varint(run)
  for _, _, _, length in entries:
    out += write_varint(length)
  for i, (_, _, offset, _) in enumerate(entries):
    contiguous = i and offset == entries[i - 1][2] + entries[i - 1][3]
    out += write_varint(0 if contiguous else offset + 1)
  return gzip.compress(bytes(out), mtime=0)


def tiles_in_circle(lat: float, lon: float, radius: float, zoom: int = ZOOM) -> list[tuple[int, int]]:
  """(x, y) of the tiles at zoom that touch a circle of radius metres around (lat, lon)."""
  n = 2 ** zoom
  dlat = math.degrees(radius / EARTH_RADIUS)
  dlon = dlat / max(math.cos(math.radians(lat)), 1e-6)

  def tx(lo):
    return int((lo + 180) / 360 * n)

  def ty(la):
    return int((1 - math.asinh(math.tan(math.radians(la))) / math.pi) / 2 * n)

  def tile_lat(y):
    return math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))

  kx = EARTH_RADIUS * math.cos(math.radians(lat)) * math.pi / 180
  ky = EARTH_RADIUS * math.pi / 180
  out = []
  for y in range(max(ty(lat + dlat), 0), min(ty(lat - dlat), n - 1) + 1):
    top, bottom = tile_lat(y), tile_lat(y + 1)
    for x in range(max(tx(lon - dlon), 0), min(tx(lon + dlon), n - 1) + 1):
      left, right = x / n * 360 - 180, (x + 1) / n * 360 - 180
      nx = min(max(lon, left), right)
      ny = min(max(lat, bottom), top)
      if math.hypot((nx - lon) * kx, (ny - lat) * ky) <= radius:
        out.append((x, y))
  return out


def keep_roads(tile: bytes) -> bytes | None:
  """An MVT tile (gzip) cut to its roads layer, gzipped again; None when it has no roads."""
  raw = gzip.decompress(tile)
  kept = bytearray()
  for number, wire, value in fields(raw):
    if number == 3 and wire == 2:
      name = next((v for n, w, v in fields(value) if n == 1 and w == 2), b'')
      if name in KEEP_LAYERS:
        kept += write_varint(3 << 3 | 2) + write_varint(len(value)) + value
  return gzip.compress(bytes(kept), mtime=0) if kept else None


def _ranges(located: list[tuple[int, int, int]]) -> list[tuple[int, int, list[tuple[int, int, int]]]]:
  """Group (tile id, offset, length) sorted by offset into read requests: (start, length, members)."""
  groups = []
  for tid, offset, length in located:
    if groups:
      start, end, members = groups[-1]
      if offset - end <= MERGE_GAP and offset + length - start <= MAX_REQUEST:
        groups[-1] = (start, max(end, offset + length), members + [(tid, offset, length)])
        continue
    groups.append((offset, offset + length, [(tid, offset, length)]))
  return [(s, e - s, m) for s, e, m in groups]


def locate_all(archive: PMTiles, ids: list[int]) -> list[tuple[int, int, int]]:
  """(tile id, absolute offset, length) of each wanted tile present in the archive. Reads every leaf directory once."""
  wanted = sorted(set(ids))
  found = []

  def walk(offset, length, lo, hi, depth):
    if depth > 3:
      raise ValueError("PMTiles directory nesting exceeds four levels")
    dir_ids, entries = archive.directory(offset, length)
    for i, tid in enumerate(dir_ids):
      run, ln, off = entries[i]
      nxt = dir_ids[i + 1] if i + 1 < len(dir_ids) else 1 << 62
      if run:
        # tiles tid .. tid + run - 1 share these bytes
        for w in wanted[_bisect(wanted, tid):_bisect(wanted, min(tid + run, nxt))]:
          found.append((w, archive.data_offset + off, ln))
      elif _bisect(wanted, nxt) > _bisect(wanted, tid):
        walk(archive.leaf_offset + off, ln, tid, nxt, depth + 1)

  walk(archive.root_offset, archive.root_length, 0, 1 << 62, 0)
  return found


def _bisect(a, x):
  from bisect import bisect_left
  return bisect_left(a, x)


def extract(archive: PMTiles, lat: float, lon: float, radius: float, out_path: str,
            progress: Callable[[int, int], None] | None = None, cancelled: Callable[[], bool] | None = None,
            threads: int = 4, metadata: dict | None = None) -> dict:
  """Write the road map for the circle to out_path. Returns a summary. Raises Cancelled if cancelled() turns true."""
  if not archive.min_zoom <= ZOOM <= archive.max_zoom:
    raise ValueError(f"source has no zoom {ZOOM} tiles")
  xy = tiles_in_circle(lat, lon, radius)
  ids = {tile_id(ZOOM, x, y): (x, y) for x, y in xy}
  located = sorted(locate_all(archive, list(ids)), key=lambda t: (t[1], t[0]))
  requests = _ranges(located)
  total = sum(r[1] for r in requests)

  data_path = out_path + ".data"
  done = 0
  tiles: dict[int, tuple[int, int]] = {}  # tile id -> (offset in data, length)
  by_content: dict[bytes, tuple[int, int]] = {}
  data_len = 0
  downloaded = 0
  try:
    with open(data_path, 'wb') as data, ThreadPoolExecutor(max_workers=threads) as pool:
      for start in range(0, len(requests), threads * 4):
        batch = requests[start:start + threads * 4]
        if cancelled is not None and cancelled():
          raise Cancelled()
        for (offset, length, members), blob in zip(batch, pool.map(lambda r: archive.read(r[0], r[1]), batch), strict=True):
          downloaded += len(blob)
          cache: dict[tuple[int, int], bytes | None] = {}
          for tid, toff, tlen in members:
            key = (toff, tlen)
            if key not in cache:
              cache[key] = keep_roads(blob[toff - offset:toff - offset + tlen])
            cut = cache[key]
            if cut is None:
              continue
            if cut not in by_content:
              data.write(cut)
              by_content[cut] = (data_len, len(cut))
              data_len += len(cut)
            tiles[tid] = by_content[cut]
          done += length
          if progress is not None:
            progress(done, total)
    if not tiles:
      raise ValueError("no roads in that area")
    _write_archive(out_path, data_path, tiles, lat, lon, radius, metadata or {})
  finally:
    if os.path.exists(data_path):
      os.remove(data_path)
  return {"tiles": len(tiles), "bytes": os.path.getsize(out_path), "downloaded": downloaded, "wanted": len(ids)}


def _write_archive(out_path, data_path, tiles, lat, lon, radius, metadata):
  # entries in tile id order; identical neighbours become one run
  entries: list[list[int]] = []
  for tid in sorted(tiles):
    off, ln = tiles[tid]
    if entries and entries[-1][0] + entries[-1][1] == tid and entries[-1][2] == off:
      entries[-1][1] += 1
    else:
      entries.append([tid, 1, off, ln])
  entries_t = [tuple(e) for e in entries]
  root = encode_directory(entries_t)
  leaves = b''
  if len(root) > ROOT_MAX:
    size = LEAF_ENTRIES
    while True:
      chunks = [entries_t[i:i + size] for i in range(0, len(entries_t), size)]
      blobs = [encode_directory(c) for c in chunks]
      pointers, offset = [], 0
      for c, b in zip(chunks, blobs, strict=True):
        pointers.append((c[0][0], 0, offset, len(b)))
        offset += len(b)
      root = encode_directory(pointers)
      if len(root) <= ROOT_MAX:
        leaves = b''.join(blobs)
        break
      size *= 2

  meta = gzip.compress(json.dumps({"name": "FM Pilot Nav Assist roads", "attribution": "© OpenStreetMap contributors",
                                   "fm_center": [lat, lon], "fm_radius_m": radius, **metadata}).encode(), mtime=0)
  data_len = os.path.getsize(data_path)
  root_off = HEADER_LENGTH
  meta_off = root_off + len(root)
  leaf_off = meta_off + len(meta)
  data_off = leaf_off + len(leaves)
  dlat = math.degrees(radius / EARTH_RADIUS)
  dlon = dlat / max(math.cos(math.radians(lat)), 1e-6)
  e7 = lambda v: int(round(v * 1e7))  # noqa: E731
  header = bytearray(HEADER_LENGTH)
  header[:8] = b'PMTiles\x03'
  struct.pack_into('<11Q', header, 8, root_off, len(root), meta_off, len(meta), leaf_off, len(leaves), data_off, data_len,
                   len(tiles), len(entries_t), len(set(tiles.values())))
  header[96:102] = bytes([1, 2, 2, 1, ZOOM, ZOOM])  # clustered, gzip directories, gzip tiles, MVT, zoom 15 only
  struct.pack_into('<4i', header, 102, e7(lon - dlon), e7(lat - dlat), e7(lon + dlon), e7(lat + dlat))
  struct.pack_into('<B2i', header, 118, ZOOM, e7(lon), e7(lat))
  with open(out_path, 'wb') as f, open(data_path, 'rb') as d:
    f.write(header)
    f.write(root)
    f.write(meta)
    f.write(leaves)
    while chunk := d.read(1 << 20):
      f.write(chunk)


class HttpSource:
  """A remote PMTiles archive read with HTTP range requests (for PMTiles)."""
  def __init__(self, url: str, timeout: float = 30.0, retries: int = 3):
    import requests
    self.url, self.timeout, self.retries = url, timeout, retries
    self.session = requests.Session()
    r = self.session.head(url, timeout=timeout, allow_redirects=True)
    r.raise_for_status()
    self.size = int(r.headers['Content-Length'])

  def read(self, offset: int, length: int) -> bytes:
    last = None
    for _ in range(self.retries):
      try:
        r = self.session.get(self.url, headers={"Range": f"bytes={offset}-{offset + length - 1}"}, timeout=self.timeout)
        if r.status_code == 206 and len(r.content) == length:
          return r.content
        last = ValueError(f"range request returned {r.status_code}, {len(r.content)} of {length} bytes")
      except Exception as e:  # network errors: retry
        last = e
    raise last

  def prefetch(self, ranges) -> None:
    pass

  def close(self) -> None:
    self.session.close()


def check_archive(path: str, lat: float, lon: float) -> int:
  """Open a written archive and read the tile under (lat, lon); returns how many roads it has there."""
  from openpilot.sunnypilot.fm_nav.pmtiles import LocalFile
  from openpilot.sunnypilot.fm_nav.vector_tile import decode_roads
  a = PMTiles(LocalFile(path))
  try:
    n = 2 ** ZOOM
    x = int((lon + 180) / 360 * n)
    y = int((1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n)
    found = 0
    for dx in (-1, 0, 1):
      for dy in (-1, 0, 1):
        t = a.get(ZOOM, x + dx, y + dy)
        found += len(decode_roads(t)) if t else 0
    return found
  finally:
    a.close()


def decode_check(path):
  """Every directory and tile of an archive decodes (tests)."""
  from openpilot.sunnypilot.fm_nav.pmtiles import LocalFile
  a = PMTiles(LocalFile(path))
  ids, entries = decode_directory(gzip.decompress(a.read(a.root_offset, a.root_length)))
  a.close()
  return len(ids)
