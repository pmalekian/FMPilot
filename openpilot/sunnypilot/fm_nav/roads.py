"""
FM Pilot Nav Assist: the roads around the truck, and which street a turn signal points at.

Roads come from a Protomaps basemap extract stored on the device (OpenStreetMap data, (c) OpenStreetMap contributors,
ODbL). Each road is kept as points every POINT_SPACING metres with the road's direction at that point, in local metres
east / north of a fixed origin.

`RoadMap.pick` answers one question without any destination: from here, heading this way, which road leaves the road
I'm on toward `side` within RANGE metres? It reads the map only; it never looks at the model, and it decides nothing
about steering. Service roads (parking and access roads; OpenStreetMap doesn't tell a driveway from a lot's entrance
here) are found too, but reported apart from streets.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from openpilot.sunnypilot.fm_nav.pmtiles import PMTiles, LocalFile
from openpilot.sunnypilot.fm_nav.vector_tile import decode_roads

ZOOM = 15
POINT_SPACING = 2.0  # m between stored road points
LOAD_MARGIN = 150.0  # m around the truck that must be loaded
MAX_TILES = 16  # decoded tiles kept
RANGE = 80.0  # m ahead a turn is looked for
BEHIND = 2.0  # m behind the truck a junction still counts (GPS lags a little)
NEAR = RANGE + 30.0  # m around the truck searched for other roads
TOUCH = 10.0  # m between two roads' centrelines that makes a junction (a divided road's two halves are further apart)
JUNCTION_GAP = 25.0  # m along a road between two separate junctions with the same road
CORRIDOR = (8.0, 0.25)  # a junction lies within 8 m + 25% of its distance of the line straight ahead
LEG = (12.0, 28.0)  # m from the junction where the side road's direction is read
TURN_ANGLE = (35.0, 145.0)  # deg off the truck's heading that counts as a turn to that side
ON_ROAD = 15.0  # m from a road's centreline to count as on it
ON_ROAD_ANGLE = 30.0  # deg between the truck's heading and the road's direction
EARTH_CIRCUMFERENCE = 40075016.68557849
NOT_ROADS = {'footway', 'path', 'cycleway', 'pedestrian', 'steps', 'track', 'platform', 'pier', 'raceway', 'corridor', 'bridleway'}
SERVICE = {'service', 'driveway', 'parking_aisle', 'alley'}


@dataclass
class Turn:
  distance: float  # m from the truck to the junction, along the truck's heading
  angle: float  # deg, + right, the side road's direction against the truck's heading
  name: str
  kind: str
  service: bool
  key: str = ''


@dataclass
class Picks:
  road: str = ''  # the road the truck is on ('' = none within ON_ROAD)
  road_name: str = ''
  road_kind: str = ''
  road_service: bool = False
  cross_track: float = 0.0
  streets: list[Turn] = field(default_factory=list)  # nearest first
  services: list[Turn] = field(default_factory=list)


def _dense(p: np.ndarray):
  """Each segment of a polyline as (points every POINT_SPACING, the segment's direction in deg, 0-180)."""
  for a, b in zip(p[:-1], p[1:], strict=True):
    d = b - a
    length = float(np.hypot(*d))
    if length < 1e-3:
      continue
    t = np.linspace(0, 1, max(2, int(length / POINT_SPACING) + 1))
    yield a + t[:, None] * d, np.full(len(t), math.degrees(math.atan2(d[0], d[1])) % 180)


class _Tile:
  def __init__(self, xy, bearing, road, poly, keys, names, kinds):
    self.xy, self.bearing, self.road, self.poly = xy, bearing, road, poly
    self.keys, self.names, self.kinds = keys, names, kinds


class RoadMap:
  def __init__(self, path: str):
    self.archive = PMTiles(LocalFile(path))
    self.zoom = min(ZOOM, self.archive.max_zoom)
    self.origin = None  # (lon, lat) of local (0, 0)
    self.tiles: dict[tuple[int, int], _Tile | None] = {}
    self.active: tuple = ()
    self.xy = np.empty((0, 2))
    self.bearing = np.empty(0)
    self.road = np.empty(0, dtype=np.int32)
    self.poly = np.empty(0, dtype=np.int32)
    self.keys: list[str] = []
    self.names: list[str] = []
    self.kinds: list[str] = []
    self.service = np.empty(0, dtype=bool)

  @classmethod
  def from_roads(cls, roads: list[tuple]) -> "RoadMap":
    """A map from roads given in local metres, [(points, name, kind)], with no archive behind it (tests)."""
    m = cls.__new__(cls)
    m.archive, m.zoom, m.tiles, m.active = None, ZOOM, {}, ()
    m.set_origin(0.0, 0.0)
    xy, bearing, road, poly, keys, names, kinds = [], [], [], [], [], [], []
    for i, (points, name, kind) in enumerate(roads):
      keys.append(name or f'#{i}')
      names.append(name)
      kinds.append(kind)
      for part in _dense(np.asarray(points, dtype=np.float64)):
        xy.append(part[0])
        bearing.append(part[1])
        road.append(np.full(len(part[0]), i, dtype=np.int32))
        poly.append(np.full(len(part[0]), i, dtype=np.int32))
    m._join([_Tile(np.concatenate(xy), np.concatenate(bearing), np.concatenate(road), np.concatenate(poly), keys, names, kinds)])
    return m

  def close(self) -> None:
    if self.archive is not None:
      self.archive.close()

  # *** coordinates ***
  def set_origin(self, lon: float, lat: float) -> None:
    self.origin = (lon, lat)
    self.kx = EARTH_CIRCUMFERENCE / 360 * math.cos(math.radians(lat))
    self.ky = 110574.0
    self.tiles.clear()
    self.active = ()

  def local(self, lon, lat) -> np.ndarray:
    return np.column_stack(((np.asarray(lon, dtype=np.float64) - self.origin[0]) * self.kx,
                            (np.asarray(lat, dtype=np.float64) - self.origin[1]) * self.ky))

  def _tile_xy(self, lon: float, lat: float) -> tuple[float, float]:
    n = 2 ** self.zoom
    return (lon + 180) / 360 * n, (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n

  # *** loading ***
  def _load(self, tx: int, ty: int) -> _Tile | None:
    data = self.archive.get(self.zoom, tx, ty)
    if not data:
      return None
    n = 2 ** self.zoom
    xy, bearing, road, poly, keys, names, kinds, index = [], [], [], [], [], [], [], {}
    for i, r in enumerate(decode_roads(data)):
      if r.detail in NOT_ROADS or len(r.points) < 2:
        continue
      lon = (tx + r.points[:, 0]) / n * 360 - 180
      lat = np.degrees(np.arctan(np.sinh(np.pi * (1 - 2 * (ty + r.points[:, 1]) / n))))
      p = self.local(lon, lat)
      key = r.name or f'#{tx}/{ty}/{i}'
      if key not in index:
        index[key] = len(keys)
        keys.append(key)
        names.append(r.name)
        kinds.append(r.detail or r.kind)
      for part in _dense(p):
        xy.append(part[0])
        bearing.append(part[1])
        road.append(np.full(len(part[0]), index[key], dtype=np.int32))
        poly.append(np.full(len(part[0]), i, dtype=np.int32))
    if not xy:
      return None
    return _Tile(np.concatenate(xy), np.concatenate(bearing), np.concatenate(road), np.concatenate(poly), keys, names, kinds)

  def update(self, lon: float, lat: float) -> bool:
    """Load the tiles around a position. True when the loaded roads changed."""
    if self.archive is None:
      return False
    if self.origin is None or abs(lat - self.origin[1]) > 0.3 or abs(lon - self.origin[0]) > 0.4:
      self.set_origin(lon, lat)
    dlon, dlat = LOAD_MARGIN / self.kx, LOAD_MARGIN / self.ky
    x0, y1 = self._tile_xy(lon - dlon, lat - dlat)
    x1, y0 = self._tile_xy(lon + dlon, lat + dlat)
    want = tuple(sorted((tx, ty) for tx in range(int(x0), int(x1) + 1) for ty in range(int(y0), int(y1) + 1)))
    if want == self.active:
      return False
    for t in want:
      if t not in self.tiles:
        if len(self.tiles) >= MAX_TILES:
          for old in [k for k in self.tiles if k not in want][:len(self.tiles) - MAX_TILES + 1]:
            del self.tiles[old]
        self.tiles[t] = self._load(*t)
    self.active = want
    self._join([self.tiles[t] for t in want if self.tiles[t] is not None])
    return True

  def _join(self, tiles: list[_Tile]) -> None:
    self.keys, self.names, self.kinds, index = [], [], [], {}
    xy, bearing, road, poly = [np.empty((0, 2))], [np.empty(0)], [np.empty(0, dtype=np.int32)], [np.empty(0, dtype=np.int32)]
    for n, t in enumerate(tiles):
      remap = np.empty(len(t.keys), dtype=np.int32)
      for i, key in enumerate(t.keys):
        if key not in index:
          index[key] = len(self.keys)
          self.keys.append(key)
          self.names.append(t.names[i])
          self.kinds.append(t.kinds[i])
        remap[i] = index[key]
      xy.append(t.xy)
      bearing.append(t.bearing)
      road.append(remap[t.road])
      poly.append(t.poly + n * 100000)
    self.xy, self.bearing, self.road, self.poly = np.concatenate(xy), np.concatenate(bearing), np.concatenate(road), np.concatenate(poly)
    self.service = np.array([k in SERVICE for k in self.kinds], dtype=bool)

  # *** questions ***
  def current_road(self, p: np.ndarray, heading: float) -> tuple[int, float] | None:
    """(road index, distance) of the nearest road running the truck's way, or None."""
    if not len(self.xy):
      return None
    d = np.hypot(self.xy[:, 0] - p[0], self.xy[:, 1] - p[1])
    off = np.abs((self.bearing - heading + 90) % 180 - 90)
    d = np.where(off < ON_ROAD_ANGLE, d, np.inf)
    i = int(np.argmin(d))
    return (int(self.road[i]), float(d[i])) if d[i] <= ON_ROAD else None

  def pick(self, p: np.ndarray, heading: float, side: int) -> Picks:
    """Roads leaving the truck's road toward side (+1 right, -1 left) within RANGE."""
    out = Picks()
    cur = self.current_road(p, heading)
    if cur is None:
      return out
    c = cur[0]
    out.road, out.road_name, out.road_kind, out.road_service, out.cross_track = self.keys[c], self.names[c], self.kinds[c], bool(self.service[c]), cur[1]
    h = math.radians(heading)
    fwd, right = np.array([math.sin(h), math.cos(h)]), np.array([math.cos(h), -math.sin(h)])
    rel = self.xy - p
    near = np.flatnonzero(np.hypot(rel[:, 0], rel[:, 1]) < NEAR)
    mine = near[self.road[near] == c]
    mine = mine[rel[mine] @ fwd > -BEHIND - TOUCH]
    others = near[self.road[near] != c]
    if not len(mine) or not len(others):
      return out
    dd = np.hypot(self.xy[others, None, 0] - self.xy[None, mine, 0], self.xy[others, None, 1] - self.xy[None, mine, 1]).min(axis=1)
    close = dd <= TOUCH
    gap = JUNCTION_GAP / POINT_SPACING
    for r in np.unique(self.road[others[close]]):
      sel = np.flatnonzero(close & (self.road[others] == r))
      whole = np.flatnonzero(self.road == r)
      for group in np.split(sel, np.flatnonzero((np.diff(others[sel]) > gap) | (np.diff(self.poly[others[sel]]) != 0)) + 1):
        q = self.xy[others[group[np.argmin(dd[group])]]]
        along, cross = float((q - p) @ fwd), float((q - p) @ right)
        if not -BEHIND < along < RANGE or abs(cross) > CORRIDOR[0] + CORRIDOR[1] * along:
          continue
        leg = self.xy[whole] - q
        dist = np.hypot(leg[:, 0], leg[:, 1])
        leg = leg[(dist > LEG[0]) & (dist < LEG[1])]
        if not len(leg):
          continue
        ang = (np.degrees(np.arctan2(leg[:, 0], leg[:, 1])) - heading + 180) % 360 - 180
        ang = ang[(side * ang >= TURN_ANGLE[0]) & (side * ang <= TURN_ANGLE[1])]
        if len(ang):
          service = bool(self.service[r])
          (out.services if service else out.streets).append(Turn(along, float(np.median(ang)), self.names[r], self.kinds[r], service, self.keys[r]))
    for turns in (out.streets, out.services):
      turns.sort(key=lambda t: t.distance)
      # the same road reached through two of its pieces is one junction
      turns[:] = [t for i, t in enumerate(turns) if not any(t.key == u.key and t.distance - u.distance < JUNCTION_GAP for u in turns[:i])]
    return out
