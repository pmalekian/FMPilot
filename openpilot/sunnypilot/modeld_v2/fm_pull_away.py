"""FM Pilot: Calmer Straight Pull-Away (FM_StraightPullAwayFloor).

Action-head models (Cheddar, TSFDOM ...) output a lateral accel that modeld_v2 turns into a curvature by dividing by the
speed squared, with the speed floored at ACTION_SPEED_FLOOR (1.0 m/s). A model that is not exactly zero at a standstill
(Cheddar: about -0.014 to -0.026 m/s^2, i.e. left) is therefore read as a tight curve the moment openpilot starts steering
at 0.3 m/s: the wheel is pulled to 15-63 deg and then straightened as the speed passes 1 m/s.

Calmer Crawl-Speed Steering (FM_LowSpeedCurvatureFloor) uses James Vecellio-Grant's 2.5 m/s floor from sunnypilot #2091
(MIT) throughout driving, which also softens turns from a stop and at crawl speed. Calmer Straight Pull-Away uses that
floor when pulling away straight from a stop:

- Decided at a standstill: calm if nothing says turn (blinker, turn desire, a held press, wheel past 30 deg).
- During the pull-away a blinker, a turn desire or a press held 0.3 s switches to the usual floor for the rest of it.
  The wheel angle is not checked while moving: the model's own leftover request could wind it past the limit and
  switch the floor back, which is the jerk again.
- Every switch toward the usual floor blends over 0.5 s (no one-frame jump in the request).
- Slowing into crawl speed from above (no stop) keeps the usual floor, so rolling crawl turns are as before.
- From 2.5 m/s both floors give the same curvature, so the end of the pull-away is seamless.
"""
import math

PARAM_CRAWL_SPEED = "FM_LowSpeedCurvatureFloor"
PARAM_STRAIGHT_PULL_AWAY = "FM_StraightPullAwayFloor"
ACTION_SPEED_FLOOR = 1.0  # m/s, AP's stock floor
STRAIGHT_FLOOR = 2.5       # m/s, sunnypilot #2091's floor
HANDOVER_SPEED = STRAIGHT_FLOOR  # at and above this both floors give the same curvature
STOPPED_SPEED = 0.1        # m/s, at a standstill the decision is taken afresh every frame (openpilot doesn't steer here)
TURNING_ANGLE_DEG = 30.0   # at a standstill, a wheel wound further than this is a turn in progress
PRESS_HOLD_S = 0.3         # a press must last this long to count as steering a turn (not a brush or a wheel reaction)
BLEND_S = 0.5              # calm -> usual floor over this long
DT = 0.05                  # modeld's frame (20 Hz)


class StraightPullAway:
  """Once per model frame: the speed floor the action is converted with."""

  def __init__(self):
    self.usual = 1.0      # 0 = calm floor, 1 = usual floor; in between blends the two curvatures
    self.target = 1.0
    self.press_time = 0.0
    self.base_floor = ACTION_SPEED_FLOOR
    self.enabled = False

  def read_params(self, params) -> None:
    self.base_floor = STRAIGHT_FLOOR if params.get_bool(PARAM_CRAWL_SPEED) else ACTION_SPEED_FLOOR
    self.enabled = params.get_bool(PARAM_STRAIGHT_PULL_AWAY)

  def speed_floor(self, v_ego: float, car_state, desire_active: bool) -> float:
    return self.update(self.enabled, self.base_floor, v_ego, car_state.leftBlinker or car_state.rightBlinker,
                       desire_active, car_state.steeringAngleDeg, car_state.steeringPressed)

  def update(self, enabled: bool, base_floor: float, v_ego: float, blinker: bool, desire_active: bool,
             wheel_angle_deg: float, steering_pressed: bool) -> float:
    """The floor to use: base_floor (the usual one), STRAIGHT_FLOOR, or one between (while blending)."""
    v_ego = v_ego if math.isfinite(v_ego) else 0.0
    self.press_time = self.press_time + DT if steering_pressed else 0.0
    turning = bool(blinker or desire_active or self.press_time >= PRESS_HOLD_S)

    if v_ego < STOPPED_SPEED:
      # openpilot doesn't steer at a standstill, so the decision can be taken without a blend
      at_angle = abs(wheel_angle_deg) > TURNING_ANGLE_DEG if math.isfinite(wheel_angle_deg) else True
      self.target = 1.0 if (not enabled or turning or at_angle) else 0.0
      self.usual = self.target
    elif v_ego >= HANDOVER_SPEED:
      self.target = self.usual = 1.0  # both floors agree here; a slow-down from above keeps the usual floor
    else:
      if turning or not enabled:
        self.target = 1.0
      self.usual = min(self.target, self.usual + DT / BLEND_S) if self.usual < self.target else self.target

    straight = max(STRAIGHT_FLOOR, base_floor)
    if self.usual >= 1.0 or base_floor >= straight:
      return base_floor
    # the floor that gives the blend of the two curvatures (a / max(v, f)^2); >= v_ego, so modeld's max() keeps it
    inv = self.usual / max(v_ego, base_floor) ** 2 + (1.0 - self.usual) / max(v_ego, straight) ** 2
    return 1.0 / math.sqrt(inv)
