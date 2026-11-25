"""
FM Pilot: Queue Guard.

Below 20 mph a blinker starts the turn at once (AP's Lane Turn Desire), so waiting in a line of cars to turn, every
creep forward steers toward the turn: in 000000af--3 (Nov 20, a left at a stop sign, the car ahead 2.4-5.4 m away) each
1-5 mph creep asked 0.015-0.048 1/m and wound the wheel to 79 deg, toward the median.

With the setting on, the turn waits while the truck is in a queue: it starts holding once the truck is stopped (under
0.3 m/s) during a turn with a lead within 8 m, and keeps holding while that lead stays within 10 m. While it holds, the
turn desire is withheld from everything (the model, Turn Assist, the "Turning" alert), so nothing steers toward the
turn. It lets go when the lead pulls more than 10 m ahead or is gone, the truck reaches 10 mph, or the truck has turned
10 degrees toward the turn (you are steering it yourself); the turn then comes back as a fresh turn (the model gets a
new desire pulse, Turn Assist starts a new turn). Replayed on 000000af--3 a 5 mph limit let go during a 5.5 mph creep
(+22.3 s) with the car 6 m ahead, which is when the wheel wound to 79 deg, so the limit is 10 mph.

It never starts once the truck has turned 10 degrees toward the turn since the turn desire began (a turn already
under way that stops behind a car keeps going), and it only uses the lead in the truck's path (radarState.leadOne).

While it holds, modeld logs fmModel.queueGuard (published before modelDataV2SP, so controlsd never sees the hidden
desire first), and Turn Assist (fm_turn_assist.py) takes the hold as a wait, never as the blinker cancelling mid-turn:
Finish Turn After Signal Cancels can't start a carry from it.

FM Pilot's own, ported unchanged from fm-dev-nav-settle ca08aac9 (5a112cd there).
"""
import math

PARAM = "FM_TurnQueueGuard"
STOP_SPEED = 0.3  # m/s: it only starts holding with the truck stopped (controlsd's standstill line)
QUEUE_SPEED = 10 * 0.44704  # m/s (10 mph): never holds at or above this
LEAD_CLOSE = 8.0  # m: a lead this close while stopped is a car to wait for
LEAD_CLEAR = 10.0  # m: holding ends once the lead is farther than this, or gone
TURNED_LIMIT = math.radians(10.0)  # a turn already this far along is never held
PARAMS_PERIOD_FRAMES = 50


class FmQueueGuard:
  def __init__(self, params=None):
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    self.params = params
    self.enabled = False
    self.frame = 0
    self.holding = False
    self.sign = 0  # the turn being watched: +1 right, -1 left
    self.turned = 0.0  # rad toward the turn since its desire began
    self.read_params()

  def read_params(self) -> None:
    self.enabled = self.params.get_bool(PARAM)

  def update(self, sign: int, v_ego: float, curvature: float, dt: float, lead_present: bool, lead_distance: float) -> bool:
    """True while the turn must wait. sign: the turn desire this frame (+1 right, -1 left, 0 none); curvature: the
    truck's own (controlsState.curvature, positive to the right)."""
    self.frame += 1
    if self.frame % PARAMS_PERIOD_FRAMES == 0:
      self.read_params()
    if not self.enabled or sign == 0:
      self.holding, self.sign, self.turned = False, 0, 0.0
      return False
    if sign != self.sign:
      self.sign, self.turned, self.holding = sign, 0.0, False
    self.turned += sign * curvature * max(v_ego, 0.0) * dt
    lead_close = lead_present and math.isfinite(lead_distance)
    if v_ego >= QUEUE_SPEED or self.turned >= TURNED_LIMIT:
      self.holding = False
    elif not self.holding:
      self.holding = (v_ego < STOP_SPEED and lead_close and lead_distance < LEAD_CLOSE and self.turned < TURNED_LIMIT)
    else:
      self.holding = lead_close and lead_distance < LEAD_CLEAR
    return self.holding
