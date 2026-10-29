"""
FM Pilot: road-edge turn desire.

With the setting on, a blinker toward a detected road edge (between 20 mph and the max-speed setting) becomes a turn
desire instead of a rejected lane change, and every turn desire follows one rule:

  1. Armed by the blinker. Under 20 mph that is AP's Lane Turn Desire as it stands (its own setting and speed); from
     20 mph to the max speed the road edge must also be detected on that side. No arming toward an occupied blind spot.
  2. Started by a nudge toward the blinker (or right away when lane changes are set to an automatic mode).
  3. Held until the blinker is cancelled or switched, whatever the speed does, including stops.

While a turn is armed or active, no lane change starts and no lane change prompt or road-edge message shows.
With the setting off, nothing here runs and turn desires behave exactly like stock AP.
"""
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode

TurnDirection = custom.ModelDataV2SP.TurnDirection

PARAM_ENABLED = "FM_RoadEdgeTurnDesire"
PARAM_MAX_SPEED = "FM_RoadEdgeTurnMaxSpeed"  # mph
LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
MAX_SPEED_LIMITS = (20.0, 45.0)  # mph, the setting's range
PARAMS_PERIOD_FRAMES = 50


class FmTurnDesire:
  def __init__(self, params=None):
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    self.params = params
    self.enabled = False
    self.max_speed = MAX_SPEED_LIMITS[1] * CV.MPH_TO_MS
    self.frame = 0
    self.direction = TurnDirection.none  # armed or active direction
    self.active = False
    self.read_params()

  def read_params(self) -> None:
    self.enabled = self.params.get_bool(PARAM_ENABLED)
    mph = float(self.params.get(PARAM_MAX_SPEED, return_default=True) or MAX_SPEED_LIMITS[1])
    self.max_speed = min(max(mph, MAX_SPEED_LIMITS[0]), MAX_SPEED_LIMITS[1]) * CV.MPH_TO_MS

  def reset(self) -> None:
    self.direction = TurnDirection.none
    self.active = False

  @property
  def armed(self) -> bool:
    return self.direction != TurnDirection.none

  def update(self, carstate, lateral_active: bool, stock_turn_direction, left_edge: bool, right_edge: bool,
             lane_change_mode: int) -> None:
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

    # cancelled or switched blinker ends everything
    if blinker_dir == TurnDirection.none or (self.armed and blinker_dir != self.direction):
      self.reset()
      if blinker_dir == TurnDirection.none:
        return

    if self.active:
      return  # held until the blinker is cancelled

    if not self.armed:
      left = blinker_dir == TurnDirection.turnLeft
      blindspot = carstate.leftBlindspot if left else carstate.rightBlindspot
      edge = left_edge if left else right_edge
      v_ego = carstate.vEgo
      under_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN and stock_turn_direction == blinker_dir
      edge_turn = LANE_CHANGE_SPEED_MIN <= v_ego <= self.max_speed and edge
      if not blindspot and lateral_active and (under_lane_change_speed or edge_turn):
        self.direction = blinker_dir

    if self.armed and lateral_active:
      left = self.direction == TurnDirection.turnLeft
      nudged = carstate.steeringPressed and ((carstate.steeringTorque > 0) if left else (carstate.steeringTorque < 0))
      automatic = lane_change_mode is not None and lane_change_mode not in (AutoLaneChangeMode.NUDGE, AutoLaneChangeMode.OFF)
      if nudged or automatic:
        self.active = True

  @property
  def turn_direction(self):
    return self.direction if self.active else TurnDirection.none
