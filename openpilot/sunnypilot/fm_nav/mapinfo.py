#!/usr/bin/env python3
"""
FM Pilot Nav Assist: is the map file there and does it read?

describe() opens the newest *.pmtiles in MAP_DIR the way Nav Assist does and says, in one line, what is in it: file, size,
the Protomaps build it came from, the area it covers and how many roads it has around the truck's last GPS position.
FM_NavMapInfo carries that line to Sunnylink: fm_navmapd refreshes it while parked (so it is there after a download or a
file copy), fm_navassistd once it has opened the map for a drive.

On the comma:  python3 -m openpilot.sunnypilot.fm_nav.mapinfo   (prints the same line plus the file's details)
"""
import datetime
import glob
import math
import os
import sys

from openpilot.sunnypilot.fm_nav import extract
from openpilot.sunnypilot.fm_nav.navassistd import MAP_DIR
from openpilot.sunnypilot.fm_nav.pmtiles import PMTiles, LocalFile

INFO_PARAM = "FM_NavMapInfo"
POSITION_PARAM = "LastGPSPositionLLK"
MILE = extract.MILE


def newest_map(directory: str = MAP_DIR) -> str | None:
  files = glob.glob(os.path.join(directory, "*.pmtiles"))
  return max(files, key=os.path.getmtime) if files else None


def last_position(params) -> tuple[float, float] | None:
  from openpilot.sunnypilot.fm_nav.navmapd import last_position as read
  return read(params) if params is not None else None


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  dy = math.radians(lat2 - lat1) * extract.EARTH_RADIUS
  dx = math.radians(lon2 - lon1) * extract.EARTH_RADIUS * math.cos(math.radians((lat1 + lat2) / 2))
  return math.hypot(dx, dy)


def mb(n: float) -> str:
  return f"{n / 1e6:.0f} MB" if n >= 1e6 else f"{n / 1e6:.1f} MB"


def describe(directory: str = MAP_DIR, pos: tuple[float, float] | None = None) -> tuple[bool, str]:
  """(usable, one line). usable means the file reads and has roads at pos (or pos is not known)."""
  path = newest_map(directory)
  if path is None:
    return False, "No map. Turn on Download Nav Map while parked with internet, or copy a .pmtiles file into /data/media/0/fmnav."
  name, size = os.path.basename(path), os.path.getsize(path)
  try:
    archive = PMTiles(LocalFile(path))
    try:
      meta = archive.metadata()
    finally:
      archive.close()
  except Exception as e:
    return False, f"Can't read {name} ({mb(size)}): {str(e)[:80]}. Download it again."
  parts = [name, mb(size)]
  if meta.get("fm_build"):
    b = str(meta["fm_build"])
    parts.append(f"map of {b[:4]}-{b[4:6]}-{b[6:]}")
  center, radius = meta.get("fm_center"), meta.get("fm_radius_m")
  if center and radius:
    parts.append(f"{radius / MILE:.0f} mi around {center[0]:.3f}, {center[1]:.3f}")
  if pos is None:
    return True, "Map OK: " + ", ".join(parts) + ". No GPS position saved yet to check the roads against."
  try:
    roads = extract.check_archive(path, pos[0], pos[1])
  except Exception as e:
    return False, f"Can't read roads from {name}: {str(e)[:80]}. Download it again."
  outside = ""
  if center and radius:
    d = distance_m(center[0], center[1], pos[0], pos[1])
    if d > radius:
      outside = f" The truck's last position is {d / MILE:.0f} mi from its center, outside it."
  if roads == 0:
    return False, "Map has no roads where the truck last was: " + ", ".join(parts) + "." + outside
  return True, f"Map OK: {', '.join(parts)}. {roads} road pieces around the truck's last position.{outside}"


def publish(params, directory: str = MAP_DIR) -> str:
  """Write the line to FM_NavMapInfo (only when it changed) and return it."""
  _, text = describe(directory, last_position(params))
  if params.get(INFO_PARAM) != text:
    params.put(INFO_PARAM, text)
  return text


def signature(params, directory: str = MAP_DIR) -> tuple:
  """Changes when the map file or the saved position does, so describe() runs only then."""
  path = newest_map(directory)
  st = os.stat(path) if path else None
  raw = params.get(POSITION_PARAM)
  return (path, st.st_mtime_ns if st else 0, st.st_size if st else 0, raw if isinstance(raw, (str, bytes)) else None)


def main() -> None:
  ok, text = describe(MAP_DIR, _cli_position())
  print(text)
  path = newest_map()
  if path:
    print(f"file: {path}, modified {datetime.datetime.fromtimestamp(os.path.getmtime(path)):%Y-%m-%d %H:%M}")
  sys.exit(0 if ok else 1)


def _cli_position():
  from openpilot.common.params import Params
  return last_position(Params())


if __name__ == "__main__":
  main()
