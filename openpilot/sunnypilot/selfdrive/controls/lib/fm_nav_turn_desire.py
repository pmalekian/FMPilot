"""
FM Pilot: Map Turn Desire.

Above the lane-turn speed a blinker means a lane change. With this on, a blinker toward a street the stored map shows
on that side (Nav Assist, openpilot/sunnypilot/fm_nav) means a turn instead:

  1. From the lane-turn speed (20 mph) to MAX_SPEED, while the map shows a street leaving the truck's road toward the
     blinker within range, no lane change starts ("armed").
  2. Once that junction is within about 6 s of travel (at least 35 m, at most the map's 80 m), the turn desire starts,
     if openpilot is steering and no car is in that blind spot. It is held until the blinker is cancelled or switched.
  3. A blinker that has already changed lanes stays a lane-change blinker: no map turn until it is cancelled. (Nov 3
     log: a lane change at 42 mph with the blinker left on while braking for a light, a side street 80 m ahead.)
  4. Under 20 mph nothing changes: AP's Lane Turn Desire starts turns as before.

Only streets count; service roads (parking and access roads) never start a turn here. Without a fresh map message, or
with the setting off, nothing here runs. The driver's blinker is always required.
"""
from openpilot.cereal import custom
from openpilot.common.constants import CV

TurnDirection = custom.ModelDataV2SP.TurnDirection

PARAM_ENABLED = "FM_NavTurnDesire"
LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
MAX_SPEED = 35 * CV.MPH_TO_MS
START_TIME = 6.0  # s of travel to the junction at which the turn desire starts
START_DISTANCE = (35.0, 80.0)  # m, limits of that distance
PARAMS_PERIOD_FRAMES = 50


def start_distance(v_ego: float) -> float:
  return min(max(v_ego * START_TIME, START_DISTANCE[0]), START_DISTANCE[1])


class FmNavTurnDesire:
  def __init__(self, params=None):
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    self.params = params
    self.enabled = False
    self.frame = 0
    self.direction = TurnDirection.none
    self.blinker_dir = TurnDirection.none
    self.active = False
    self.armed = False
    self.changed_lanes = False  # this blinker has already been used for a lane change
    self.read_params()

  def read_params(self) -> None:
    self.enabled = self.params.get_bool(PARAM_ENABLED)

  def reset(self) -> None:
    self.direction = TurnDirection.none
    self.active = False
    self.armed = False

  def update(self, carstate, lateral_active: bool, nav, lane_changing: bool = False) -> None:
    """nav: a fresh fmNavTurn message, or None."""
    self.frame += 1
    if self.frame % PARAMS_PERIOD_FRAMES == 0:
      self.read_params()
    if not self.enabled:
      self.reset()
      return

    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    blinker_dir = TurnDirection.none
    if one_blinker:
      blinker_dir = TurnDirection.turnLeft if carstate.leftBlinker else TurnDirection.turnRight

    # a cancelled or switched blinker ends everything
    if blinker_dir != self.blinker_dir:
      self.reset()
      self.changed_lanes = False
      self.blinker_dir = blinker_dir
    if blinker_dir == TurnDirection.none:
      return
    self.changed_lanes = self.changed_lanes or lane_changing

    if self.active:
      return  # held until the blinker is cancelled

    self.armed = False
    self.direction = TurnDirection.none
    v_ego = carstate.vEgo
    if nav is None or str(nav.status) != 'ok' or self.changed_lanes or not LANE_CHANGE_SPEED_MIN <= v_ego <= MAX_SPEED:
      return
    left = blinker_dir == TurnDirection.turnLeft
    pick = nav.left if left else nav.right
    if not pick.valid:
      return
    self.direction = blinker_dir
    self.armed = True
    blindspot = carstate.leftBlindspot if left else carstate.rightBlindspot
    if lateral_active and not blindspot and pick.distance <= start_distance(v_ego):
      self.active = True

  @property
  def turn_direction(self):
    return self.direction if self.active else TurnDirection.none
