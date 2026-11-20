from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController
from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_desire import FmTurnDesire
from openpilot.sunnypilot.selfdrive.controls.lib.fm_turn_pulse import FmTurnPulse
from openpilot.sunnypilot.selfdrive.controls.lib.fm_nav_turn_desire import FmNavTurnDesire
from openpilot.sunnypilot.selfdrive.controls.lib.fm_queue_guard import FmQueueGuard

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.model_desire = log.Desire.none  # what modeld feeds the model; FM Pilot's Turn Pulse on Pull-Away can differ
    self.alc = AutoLaneChangeController(self)
    self.lane_turn_controller = LaneTurnController(self)
    self.lane_turn_direction = TurnDirection.none
    self.fm_turn = FmTurnDesire()  # FM Pilot: road-edge turn desire and the held-turn rule
    self.fm_turn_pulse = FmTurnPulse()  # FM Pilot: fresh turn pulse to the model when pulling away from a stop
    self.fm_nav_turn = FmNavTurnDesire()  # FM Pilot: a blinker toward a mapped street is a turn, above the lane-turn speed too
    self.fm_queue_guard = FmQueueGuard()  # FM Pilot: a turn waits while stopped close behind a car
    self.fm_queue_hold = False

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False, nav_turn=None,
             lead=None, curvature=0.0):
    """lead: radarState.leadOne (or None); curvature: controlsState.curvature, both for FM Pilot's Queue Guard."""
    self.alc.update_params()
    self.lane_turn_controller.update_params()
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # Lane turn controller update
    self.lane_turn_controller.update_lane_turn(blindspot_left=carstate.leftBlindspot, blindspot_right=carstate.rightBlindspot,
                                               left_blinker=carstate.leftBlinker, right_blinker=carstate.rightBlinker, v_ego=v_ego)
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    # FM Pilot: with road-edge turn desires on, turns start on the blinker (no nudge) and are held until the blinker
    # is cancelled; an active turn takes the place of a lane change
    self.fm_turn.update(carstate, lateral_active, self.lane_turn_direction, left_edge_detected, right_edge_detected)
    fm_turn_armed = False
    if self.fm_turn.enabled:
      self.lane_turn_direction = self.fm_turn.turn_direction
      fm_turn_armed = self.fm_turn.armed

    # FM Pilot: Map Turn Desire. A street on the map toward the blinker holds off the lane change and, close enough,
    # starts the turn. It never replaces a turn already chosen above.
    lane_changing = self.lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)
    self.fm_nav_turn.update(carstate, lateral_active, nav_turn, lane_changing)
    if self.fm_nav_turn.enabled and self.lane_turn_direction == TurnDirection.none:
      self.lane_turn_direction = self.fm_nav_turn.turn_direction
      fm_turn_armed = fm_turn_armed or self.fm_nav_turn.armed

    # FM Pilot: Queue Guard. Stopped close behind a car, the turn waits (withheld from the model, Turn Assist and the
    # alert) until that car pulls away; then it comes back as a fresh turn.
    turn_sign = 1 if self.lane_turn_direction == TurnDirection.turnRight else (-1 if self.lane_turn_direction == TurnDirection.turnLeft else 0)
    lead_present = bool(lead is not None and lead.present)
    self.fm_queue_hold = self.fm_queue_guard.update(turn_sign, v_ego, float(curvature), DT_MDL, lead_present,
                                                    float(lead.dRel) if lead_present else float('inf'))
    if self.fm_queue_hold:
      self.lane_turn_direction = TurnDirection.none

    if fm_turn_armed or not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or \
       self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = (((carstate.leftBlindspot or left_edge_detected) and self.lane_change_direction == LaneChangeDirection.left) or
                              ((carstate.rightBlindspot or right_edge_detected) and self.lane_change_direction == LaneChangeDirection.right))

        self.alc.update_lane_change(blindspot_detected, carstate.brakePressed)

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        elif (torque_applied or self.alc.auto_lane_change_allowed) and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

    self.prev_one_blinker = one_blinker and lateral_active

    if self.lane_turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.lane_turn_direction]
    else:
      self.desire = log.Desire.none
      if self.lane_change_state == LaneChangeState.laneChangeStarting:
        if self.lane_change_direction == LaneChangeDirection.left:
          self.desire = log.Desire.laneChangeLeft
        elif self.lane_change_direction == LaneChangeDirection.right:
          self.desire = log.Desire.laneChangeRight

    self.model_desire = self.fm_turn_pulse.update(self.desire, v_ego)

    self.alc.update_state()
