#!/usr/bin/env python3
"""
FM Pilot Nav Assist process: publishes fmNavTurn, the next road leaving the truck's road to each side.

Reads GPS, speed and yaw rate and a road map stored on the device (the newest *.pmtiles in MAP_DIR: Download Nav Map's
fm-nav.pmtiles, or a copied Protomaps extract). It sends nothing to the car and nothing to the model; whether anything acts on its message is another
setting's business (Map Turn Desire). It runs on the small cores at normal priority, away from the control loops.

No map file: it keeps publishing status noMap, so the log says why nothing was found.
"""
import glob
import os
import time

import openpilot.cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.fm_nav.core import NavAssist, State
from openpilot.sunnypilot.fm_nav.roads import RoadMap, Turn

MAP_DIR = os.environ.get("FM_NAV_MAP_DIR", "/data/media/0/fmnav")
PUBLISH_PERIOD = 0.2  # s; the loop itself runs on every yaw rate message (20 Hz)
SMALL_CORES = (0, 1, 2, 3)
MAP_RETRY_S = 30.0


def open_map(directory: str = MAP_DIR) -> RoadMap | None:
  """The newest readable *.pmtiles in the folder (Download Nav Map writes fm-nav.pmtiles; a copied file works too)."""
  for path in sorted(glob.glob(os.path.join(directory, "*.pmtiles")), key=os.path.getmtime, reverse=True):
    try:
      return RoadMap(path)
    except Exception:
      cloudlog.exception(f"fm_navassistd: can't read {path}")
  return None


def fill_pick(pick, turns: list[Turn]) -> None:
  if not turns:
    return
  t = turns[0]
  pick.valid = True
  pick.distance = float(t.distance)
  pick.angle = float(t.angle)
  pick.name = t.name
  pick.kind = t.kind
  pick.count = min(len(turns), 255)
  pick.nextGap = float(turns[1].distance - t.distance) if len(turns) > 1 else 0.0


def fill_message(m, s: State, compute_ms: float) -> None:
  m.status = s.status
  m.heading = float(s.heading)
  m.gpsAccuracy = float(s.gps_accuracy)
  m.roadName = s.left.road_name
  m.roadKind = s.left.road_kind
  m.onService = s.left.road_service
  m.crossTrack = float(s.left.cross_track)
  m.tiles = min(s.tiles, 255)
  m.computeMs = float(compute_ms)
  fill_pick(m.left, s.left.streets)
  fill_pick(m.right, s.right.streets)
  fill_pick(m.leftService, s.left.services)
  fill_pick(m.rightService, s.right.services)


def main() -> None:
  try:
    os.sched_setaffinity(0, SMALL_CORES)
    os.nice(10)
  except (AttributeError, OSError):
    pass

  pm = messaging.PubMaster(["fmNavTurn"])
  sm = messaging.SubMaster(["liveLocationKalman", "gpsLocationExternal", "carState"], poll="liveLocationKalman")
  nav = NavAssist(open_map())
  next_map_try = time.monotonic() + MAP_RETRY_S
  last_publish = 0.0
  failures = 0

  while True:
    sm.update(100)
    now = time.monotonic()
    if sm.updated["liveLocationKalman"]:
      w = sm["liveLocationKalman"].angularVelocityCalibrated
      if w.valid and len(w.value) == 3:
        nav.on_pose(sm.logMonoTime["liveLocationKalman"] * 1e-9, w.value[2])
    if sm.updated["carState"]:
      nav.on_car(sm["carState"].vEgo)
    if sm.updated["gpsLocationExternal"]:
      g = sm["gpsLocationExternal"]
      if g.hasFix:
        nav.on_gps(sm.logMonoTime["gpsLocationExternal"] * 1e-9, g.longitude, g.latitude, g.speed, g.bearingDeg, g.horizontalAccuracy)

    if now - last_publish >= PUBLISH_PERIOD - 0.01:
      last_publish = now
      if nav.map is None and now > next_map_try:
        nav.map = open_map()
        next_map_try = now + MAP_RETRY_S
      t0 = time.perf_counter()
      try:
        state = nav.tick(now)
      except Exception:  # a damaged tile must not end the process, or take the message's rate with it
        failures += 1
        if failures <= 3:
          cloudlog.exception("fm_navassistd: tick failed")
        state = State(status="noMap")
      msg = messaging.new_message("fmNavTurn")
      msg.valid = state.status == "ok"
      fill_message(msg.fmNavTurn, state, (time.perf_counter() - t0) * 1e3)
      pm.send("fmNavTurn", msg)


if __name__ == "__main__":
  main()
