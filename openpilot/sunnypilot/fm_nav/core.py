"""
FM Pilot Nav Assist: where the truck is on the map, and the next turn to each side.

Position is the last GPS fix carried forward by speed and heading. Heading is the yaw rate summed up, tied to the GPS
course whenever the truck moves faster than HEADING_MIN_SPEED (GPS course is useless at a crawl and at a stop, which is
where turns start). Nothing here uses a destination or the driving model.

`NavAssist` holds no cereal types, so logs can be replayed through it off the device (tests/replay.py).
"""
import math
from dataclasses import dataclass, field

import numpy as np

from openpilot.sunnypilot.fm_nav.roads import RoadMap, Picks, Turn

HEADING_MIN_SPEED = 3.0  # m/s, GPS course trusted above this
HEADING_GAIN = 0.03  # share of each fix's course taken into the heading offset (10 Hz fixes: ~3 s)
GPS_MAX_AGE = 2.0  # s without a fix before the position is dropped
GPS_MAX_ACCURACY = 10.0  # m reported horizontal accuracy
MAX_SPEED = 20.0  # m/s (45 mph): no turns are looked for above this

STATUS = ('ok', 'noMap', 'noFix', 'noHeading', 'offRoad', 'tooFast')


@dataclass
class State:
  status: str = 'noFix'
  heading: float = 0.0  # deg clockwise from north
  gps_accuracy: float = 0.0
  left: Picks = field(default_factory=Picks)
  right: Picks = field(default_factory=Picks)
  tiles: int = 0

  @staticmethod
  def first(turns: list[Turn]) -> Turn | None:
    return turns[0] if turns else None


class NavAssist:
  def __init__(self, road_map: RoadMap | None):
    self.map = road_map
    self.yaw_sum = 0.0  # deg, summed yaw rate (+ right)
    self.pose_t = None
    self.offset = None  # complex unit vector: GPS course minus yaw_sum
    self.fix = None  # (t, lon, lat, accuracy)
    self.v = 0.0
    self.state = State(status='noMap' if road_map is None else 'noFix')

  def on_pose(self, t: float, yaw_rate: float) -> None:
    """yaw_rate: rad/s about the calibrated z axis (+ right)."""
    if self.pose_t is not None and math.isfinite(yaw_rate):
      self.yaw_sum += math.degrees(yaw_rate) * min(max(t - self.pose_t, 0.0), 0.2)
    self.pose_t = t

  def on_car(self, v_ego: float) -> None:
    self.v = v_ego

  def on_gps(self, t: float, lon: float, lat: float, speed: float, bearing: float, accuracy: float) -> None:
    if not (math.isfinite(lon) and math.isfinite(lat)) or accuracy > GPS_MAX_ACCURACY:
      return
    self.fix = (t, lon, lat, accuracy)
    if speed > HEADING_MIN_SPEED and math.isfinite(bearing):
      e = complex(math.cos(math.radians(bearing - self.yaw_sum)), math.sin(math.radians(bearing - self.yaw_sum)))
      self.offset = e if self.offset is None else (1 - HEADING_GAIN) * self.offset + HEADING_GAIN * e

  @property
  def heading(self) -> float | None:
    if self.offset is None:
      return None
    return (self.yaw_sum + math.degrees(math.atan2(self.offset.imag, self.offset.real))) % 360

  def tick(self, t: float) -> State:
    s = State()
    self.state = s
    if self.map is None:
      s.status = 'noMap'
      return s
    if self.fix is None or t - self.fix[0] > GPS_MAX_AGE:
      s.status = 'noFix'
      return s
    s.gps_accuracy = self.fix[3]
    heading = self.heading
    if heading is None:
      s.status = 'noHeading'
      return s
    s.heading = heading
    self.map.update(self.fix[1], self.fix[2])
    s.tiles = len(self.map.active)
    if self.v > MAX_SPEED:
      s.status = 'tooFast'
      return s
    # the fix, carried forward to now
    age = min(max(t - self.fix[0], 0.0), GPS_MAX_AGE)
    h = math.radians(heading)
    p = self.map.local(self.fix[1], self.fix[2])[0] + self.v * age * np.array([math.sin(h), math.cos(h)])
    s.left, s.right = self.map.pick(p, heading, -1), self.map.pick(p, heading, 1)
    s.status = 'ok' if s.left.road else 'offRoad'
    return s
