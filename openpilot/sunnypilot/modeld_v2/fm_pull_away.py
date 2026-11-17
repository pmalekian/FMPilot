"""FM Pilot: Calmer Straight Pull-Away (FM_StraightPullAwayFloor).

Action-head models (Cheddar, TSFDOM ...) output a lateral accel that modeld_v2 turns into a curvature by dividing by the
speed squared, with the speed floored at ACTION_SPEED_FLOOR (1.0 m/s). A model that is not exactly zero at a standstill
(Cheddar: about -0.014 to -0.026 m/s^2, i.e. left) is therefore read as a tight curve the moment openpilot starts steering
at 0.3 m/s: the wheel is pulled to 15-63 deg and then straightened as the speed passes 1 m/s.

sunnypilot #2091 (FM_LowSpeedCurvatureFloor) raises the floor to 2.5 m/s for all driving, which also softens real turns
from a stop. This module raises it only for a straight pull-away: whenever nothing says the driver is turning, and the
usual floor everywhere else. At 2.5 m/s and above both floors are identical, so the hand-over is seamless.
"""

STRAIGHT_FLOOR = 2.5       # m/s, sunnypilot #2091's floor
HANDOVER_SPEED = 2.5       # m/s, at and above this both floors give the same curvature
STOPPED_SPEED = 0.1        # m/s, at a standstill the decision is taken afresh every frame
TURNING_ANGLE_DEG = 30.0   # a wheel wound further than this is a turn in progress (stopped mid-turn, no blinker)


class StraightPullAway:
  """Decides, once per model frame, which speed floor the action is converted with.

  A turn is any of: a blinker, a turn desire (the model's or Map Turn Desire's), the wheel wound past TURNING_ANGLE_DEG,
  or the driver pressing on the wheel. While one is true below HANDOVER_SPEED the usual floor is kept, and it stays so
  until the truck passes HANDOVER_SPEED (the blinker clicking off mid-turn must not drop the request) or stops again.
  """

  def __init__(self):
    self.turning = False

  def update(self, enabled: bool, base_floor: float, v_ego: float, blinker: bool, desire_active: bool,
             wheel_angle_deg: float, steering_pressed: bool) -> float:
    """The floor to use: base_floor (the usual one) or STRAIGHT_FLOOR."""
    if not enabled or base_floor >= STRAIGHT_FLOOR:
      self.turning = False
      return base_floor
    trigger = bool(blinker or desire_active or steering_pressed or abs(wheel_angle_deg) > TURNING_ANGLE_DEG)
    if v_ego >= HANDOVER_SPEED:
      self.turning = False  # both floors agree from here; the next crawl starts afresh
    elif v_ego < STOPPED_SPEED:
      self.turning = trigger
    else:
      self.turning = self.turning or trigger
    return base_floor if (self.turning or v_ego >= HANDOVER_SPEED) else STRAIGHT_FLOOR
