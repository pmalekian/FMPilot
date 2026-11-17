"""
FM Pilot: Nav Turn Speed.

While a turn is planned (a blinker turn desire, from AP's Lane Turn Desire or Map Turn Desire) toward a street Nav Assist
shows on that side, openpilot's target speed is capped so the truck reaches Nav Turn Speed (10-20 mph, default 14) at
the point where it starts to turn in (about TURN_IN_BEFORE metres before the map's junction), slowing at most about
A_PLAN, and then held at that speed through the turn until the truck has turned TURN_DONE_DEG, the junction is
PASSED_BY metres behind without a turn (3x that in any case), or the turn desire ends.

It only ever lowers the cruise target (one more source in sunnypilot's min() of speed targets, like Smart Cruise
Control), so a lead, a stop or the model asking for less still win, and it never adds speed (from a stop it doesn't
make the truck pull away any faster). Only while openpilot controls speed. Service roads (parking and access roads)
never count. StarPilot's nav turn speeds (`starpilot/navigation/intent.py`, MIT) gave the idea and the default; no code
taken.
"""
import math

from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET

TurnDirection = custom.ModelDataV2SP.TurnDirection

PARAM_ENABLED = "FM_NavTurnSpeed"
PARAM_SPEED = "FM_NavTurnSpeedMph"
SPEED_RANGE = (10, 20)  # mph
DEFAULT_SPEED = 14
A_PLAN = 1.0  # m/s^2, the deceleration the target speed profile assumes
A_MIN = -1.2  # m/s^2, the planner's cruise limit (A_CRUISE_MIN)
TURN_IN_BEFORE = 10.0  # m before the map's junction where the turn speed must be reached (his turn-ins: 6-19 m)
TURN_DONE_DEG = 60.0  # deg turned since the cap engaged: the turn is done
PASSED_BY = 20.0  # m past the junction without turning: not this street
NAV_MAX_AGE = 0.5  # s, fmNavTurn is 5 Hz
DT = 0.05  # s, the planner's step
CRUISE_LAG = 1.0  # s: the cruise accel is (target - speed) per second, so a falling target is followed ~1 s late
PARAMS_PERIOD = 1.0  # s


def turn_speed_limit(distance_to_turn_in: float, v_turn: float) -> float:
  """Highest speed from which the truck reaches v_turn at the turn-in point slowing at A_PLAN."""
  return math.sqrt(v_turn ** 2 + 2.0 * A_PLAN * max(distance_to_turn_in, 0.0))


class FmNavTurnSpeed:
  def __init__(self, params=None):
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    self.params = params
    self.enabled = False
    self.v_turn = DEFAULT_SPEED * CV.MPH_TO_MS
    self.params_t = -1e9
    self.read_params()
    self.reset()

  def read_params(self) -> None:
    self.enabled = self.params.get_bool(PARAM_ENABLED)
    try:
      mph = int(self.params.get(PARAM_SPEED, return_default=True))
    except (TypeError, ValueError):
      mph = DEFAULT_SPEED
    self.v_turn = min(max(mph, SPEED_RANGE[0]), SPEED_RANGE[1]) * CV.MPH_TO_MS

  def reset(self) -> None:
    self.active = False  # a turn is being capped
    self.direction = TurnDirection.none
    self.distance = 0.0  # m to the junction, carried forward between map messages
    self.nav_t = None  # time of the last map message used
    self.heading0 = None  # heading when the cap engaged
    self.turned = 0.0
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.0

  def update(self, t: float, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             turn_direction, nav, nav_t: float | None) -> None:
    """turn_direction: modelDataV2SP.laneTurnDirection. nav: the latest fmNavTurn (or None), received at nav_t."""
    if t - self.params_t >= PARAMS_PERIOD:
      self.params_t = t
      self.read_params()
    self.output_v_target, self.output_a_target = V_CRUISE_UNSET, a_ego
    if not self.enabled or not long_enabled or long_override or turn_direction == TurnDirection.none or \
       (self.active and turn_direction != self.direction):
      self.reset()
      return

    fresh = nav is not None and nav_t is not None and t - nav_t <= NAV_MAX_AGE
    heading_ok = fresh and str(nav.status) not in ('noFix', 'noHeading', 'noMap')
    if self.active:
      self.distance -= v_ego * DT  # carried forward; a new map message below replaces it
    if fresh and nav_t != self.nav_t:
      self.nav_t = nav_t
      pick = nav.left if turn_direction == TurnDirection.turnLeft else nav.right
      if str(nav.status) == 'ok' and pick.valid and (not self.active or pick.distance < self.distance + 15.0):
        # the next street that way; once capping, ignore a jump to a different street further on
        if not self.active:
          self.active, self.direction = True, turn_direction
          self.heading0 = nav.heading if heading_ok else None
        self.distance = float(pick.distance)

    if not self.active:
      return
    if heading_ok:
      if self.heading0 is None:
        self.heading0 = nav.heading
      self.turned = abs((nav.heading - self.heading0 + 180.0) % 360.0 - 180.0)
    if self.turned >= TURN_DONE_DEG or (self.distance < -PASSED_BY and self.turned < 30.0) or self.distance < -3 * PASSED_BY:
      self.reset()
      return

    # the profile, lowered by what the cruise accel lags it, never below the turn speed
    self.output_v_target = max(turn_speed_limit(self.distance - TURN_IN_BEFORE, self.v_turn) - A_PLAN * CRUISE_LAG, self.v_turn)
    if v_ego > self.output_v_target:
      need = (v_ego ** 2 - self.v_turn ** 2) / (2.0 * max(self.distance - TURN_IN_BEFORE, 1.0))
      self.output_a_target = max(min(a_ego, -need), A_MIN)
