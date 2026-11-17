#!/usr/bin/env python3
"""
FM Pilot Nav Assist: Download Nav Map.

Runs only while parked. When FM_NavMapDownload is turned on (Sunnylink), it downloads the roads within RADIUS_MI miles of
where the truck last was (locationd's LastGPSPositionLLK, written once a minute while driving) from the newest daily
Protomaps world build, and saves them as MAP_DIR/fm-nav.pmtiles for Nav Assist. Progress and the result are shown in
FM_NavMapStatus. Then it turns FM_NavMapDownload off again.

If the truck starts driving, the download stops (the process only runs parked) and starts over the next time it is
parked, while the setting is still on. A new map replaces the old one only once it is complete and reads back.
"""
import datetime
import glob
import json
import os
import shutil
import time

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.fm_nav import extract
from openpilot.sunnypilot.fm_nav.navassistd import MAP_DIR
from openpilot.sunnypilot.fm_nav.pmtiles import PMTiles

RADIUS_MI = 50
BUILD_URL = "https://build.protomaps.com/{}.pmtiles"
BUILD_DAYS_BACK = 8
MAP_NAME = "fm-nav.pmtiles"
PART_NAME = ".fm-nav.pmtiles.part"
SPACE_MARGIN = 100 * 1024 * 1024
STATUS_PERIOD = 2.0
POSITION_PARAMS = ("LastGPSPositionLLK", "LastGPSPosition")


def last_position(params) -> tuple[float, float] | None:
  for key in POSITION_PARAMS:
    try:
      raw = params.get(key)
      if not raw:
        continue
      pos = json.loads(raw) if isinstance(raw, (str, bytes)) else raw
      lat, lon = float(pos["latitude"]), float(pos["longitude"])
      if -85 < lat < 85 and -180 <= lon <= 180 and (lat, lon) != (0.0, 0.0):
        return lat, lon
    except (KeyError, TypeError, ValueError):
      continue
  return None


def newest_build(open_source, today: datetime.date):
  """(date string, source) of the newest daily build that answers, looking back BUILD_DAYS_BACK days."""
  last = None
  for days in range(BUILD_DAYS_BACK + 1):
    name = (today - datetime.timedelta(days=days)).strftime("%Y%m%d")
    try:
      return name, open_source(BUILD_URL.format(name))
    except Exception as e:  # missing build or network: try the day before
      last = e
  raise RuntimeError(f"no Protomaps build found in the last {BUILD_DAYS_BACK} days ({last})")


def mb(n: float) -> str:
  return f"{n / 1e6:.0f} MB" if n >= 1e6 else f"{n / 1e6:.1f} MB"


class Downloader:
  def __init__(self, params, map_dir: str = MAP_DIR, open_source=None, now=None, onroad=None):
    self.params = params
    self.map_dir = map_dir
    self.open_source = open_source or extract.HttpSource
    self.now = now or datetime.datetime.now
    self.onroad = onroad or (lambda: params.get_bool("IsOnroad"))
    self.status_t = 0.0

  def status(self, text: str, force: bool = True) -> None:
    t = time.monotonic()
    if force or t - self.status_t >= STATUS_PERIOD:
      self.status_t = t
      self.params.put("FM_NavMapStatus", text)

  def cancelled(self) -> bool:
    return self.onroad() or not self.params.get_bool("FM_NavMapDownload")

  def run(self) -> bool:
    """One download. True when a new map is in place."""
    os.makedirs(self.map_dir, exist_ok=True)
    part = os.path.join(self.map_dir, PART_NAME)
    for stale in (part, part + ".data"):
      if os.path.exists(stale):
        os.remove(stale)
    pos = last_position(self.params)
    if pos is None:
      self.status("Failed: no position yet. Drive a minute with GPS, then try again.")
      return False
    lat, lon = pos
    where = f"{RADIUS_MI} mi around {lat:.3f}, {lon:.3f}"
    self.status(f"Finding the newest map ({where})...")
    build, source = newest_build(self.open_source, self.now().date())
    archive = PMTiles(source)
    try:
      self.status(f"Downloading {where}, map of {build[:4]}-{build[4:6]}-{build[6:]}: starting")

      def progress(done, total):
        free = shutil.disk_usage(self.map_dir).free
        if total + SPACE_MARGIN > free:
          raise RuntimeError(f"not enough space ({mb(total)} needed, {mb(free)} free)")
        self.status(f"Downloading {where}: {100 * done // max(total, 1)}% of {mb(total)}", force=False)

      summary = extract.extract(archive, lat, lon, RADIUS_MI * extract.MILE, part, progress=progress, cancelled=self.cancelled,
                                metadata={"fm_build": build})
    finally:
      archive.close()
    if extract.check_archive(part, lat, lon) == 0:
      os.remove(part)
      raise RuntimeError("the new map has no roads where the truck is")
    final = os.path.join(self.map_dir, MAP_NAME)
    os.replace(part, final)
    for other in glob.glob(os.path.join(self.map_dir, "*.pmtiles")):
      if os.path.basename(other) != MAP_NAME:
        os.remove(other)
    stamp = self.now().strftime("%Y-%m-%d %H:%M")
    day = f"{build[:4]}-{build[4:6]}-{build[6:]}"
    self.status(f"Ready: {where}, map of {day}, {mb(summary['bytes'])} ({mb(summary['downloaded'])} downloaded), {stamp}")
    cloudlog.info(f"fm_navmapd: {summary}")
    return True

  def step(self) -> None:
    if not self.params.get_bool("FM_NavMapDownload"):
      return
    try:
      self.run()
    except extract.Cancelled:
      if self.onroad():
        self.status("Stopped: started driving. It starts over when parked (setting still on).")
        return  # leave the setting on: starts over next time it's parked
      self.status("Stopped.")
    except Exception as e:
      cloudlog.exception("fm_navmapd: download failed")
      self.status(f"Failed: {e}"[:200])
    self.params.put_bool("FM_NavMapDownload", False)


def main() -> None:
  try:
    os.nice(10)
  except OSError:
    pass
  d = Downloader(Params())
  while True:
    d.step()
    time.sleep(1.0)


if __name__ == "__main__":
  main()
